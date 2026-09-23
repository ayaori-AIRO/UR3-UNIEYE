"""Zimmer GEP2006IO: user-verified DO1 open / DO2 close; no arm motion."""
import argparse
import fcntl
import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO


def run_command(io, action, hold):
    """Break before make. Cleanup is attempted even after partial failures."""
    try:
        io.set_output(1, False)
        io.set_output(2, False)
        io.pause(0.05)
        if action != 'off':
            io.set_output(1 if action == 'open' else 2, True)
            io.pause(hold)
    finally:
        errors = []
        for pin in (1, 2):
            try:
                io.set_output(pin, False)
            except Exception as exc:
                errors.append(f'DO{pin}: {exc}')
        if errors:
            raise RuntimeError('OFF NOT CONFIRMED; inspect pendant outputs: ' + '; '.join(errors))


class ZimmerIO(Node):
    def __init__(self):
        super().__init__('zimmer_gripper')
        self.client = self.create_client(SetIO, '/io_and_status_controller/set_io')
        self.states = {}
        self.updated = 0.0
        self.create_subscription(IOStates, '/io_and_status_controller/io_states',
                                 self.receive, qos_profile_sensor_data)

    def receive(self, msg):
        self.states = {int(x.pin): bool(x.state) for x in msg.digital_out_states}
        self.updated = time.monotonic()

    def set_output(self, pin, high):
        if pin not in (1, 2):
            raise ValueError('Only DO1 and DO2 are permitted')
        if high and (time.monotonic()-self.updated > 0.5
                     or self.states.get(3-pin) is not False):
            raise RuntimeError('Opposite output not freshly confirmed LOW')
        request = SetIO.Request()
        request.fun = SetIO.Request.FUN_SET_DIGITAL_OUT
        request.pin = pin
        request.state = float(high)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            # Cancellation of a local future does not cancel an in-flight robot request.
            raise RuntimeError('SetIO timeout; actual output state uncertain')
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('SetIO rejected')
        after = time.monotonic()
        deadline = after + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.updated > after and self.states.get(pin) == high:
                if high and self.states.get(3-pin) is not False:
                    raise RuntimeError('Conflicting output detected')
                return
        raise RuntimeError(f'DO{pin} feedback timeout')

    def pause(self, seconds):
        deadline = time.monotonic()+seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.02, max(0,deadline-time.monotonic())))
            if time.monotonic()-self.updated > 0.5:
                raise RuntimeError('I/O feedback stale')
            if self.states.get(1) and self.states.get(2):
                raise RuntimeError('Both outputs HIGH: external writer conflict')
        if not rclpy.ok():
            raise RuntimeError('ROS context stopped; check outputs at pendant')


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be positive and finite')
    return number


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('open', 'close', 'off'))
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--hold-time', type=positive,
                        help='required for execution: user-verified time in seconds; not a completion sensor')
    cli = parser.parse_args(rclpy.utilities.remove_ros_args(
        sys.argv if args is None else [sys.argv[0], *args])[1:])
    if cli.execute and cli.action != 'off' and cli.hold_time is None:
        parser.error('open/close execution requires --hold-time')
    print(f'Zimmer: {cli.action}; DO1=open, DO2=close, DO0 untouched; hold={cli.hold_time}')
    if not cli.execute:
        print('DRY RUN: no ROS outputs sent. Add --execute only after checking clearance.')
        return
    # Local advisory lock only; cannot exclude pendant/PLC/other PCs.
    with open('/tmp/ur3_zimmer_do1_do2.lock', 'a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('Another Zimmer command is running')
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = None
        previous = {}
        def interrupt(signum, frame):
            # Keep ROS alive for cleanup; a second interrupt should not interrupt OFF attempts.
            for sig in previous:
                signal.signal(sig, signal.SIG_IGN)
            raise KeyboardInterrupt
        try:
            for sig in (signal.SIGINT, signal.SIGTERM):
                previous[sig] = signal.signal(sig, interrupt)
            node = ZimmerIO()
            if not node.client.wait_for_service(timeout_sec=5):
                raise RuntimeError('UR set_io service unavailable; no command sent')
            run_command(node, cli.action, cli.hold_time)
            print('Output sequence complete; DO1/DO2 LOW confirmed. Jaw/grip success NOT measured.')
        except (Exception, KeyboardInterrupt) as exc:
            print(f'Command stopped: {exc}. Inspect DO1/DO2 on pendant; OFF is not an emergency stop.',
                  file=sys.stderr)
            raise SystemExit(1)
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for sig, handler in previous.items():
                signal.signal(sig, handler)

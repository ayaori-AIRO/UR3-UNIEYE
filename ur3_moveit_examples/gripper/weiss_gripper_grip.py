#!/usr/bin/env python3
"""Explicitly gated WEISS GRIPKIT XML-RPC grip test."""

import argparse
import socket
import sys
import time
import xmlrpc.client

from ur3_moveit_examples.gripper.weiss_gripper_read_state import (
    DEFAULT_DEVICE_ID,
    DEFAULT_URL,
    STATE_NAMES,
    TimeoutTransport,
    format_result,
    positive_float,
)
from ur3_moveit_examples.gripper.weiss_gripper_release import non_negative_int


EXPECTED_STATES = {
    "no-part": 4,
    "holding": 8,
}


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read the WEISS gripper state by default. With --execute, call "
            "Grip once and poll until NO_PART or HOLDING."
        )
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--index", type=non_negative_int, default=0)
    parser.add_argument(
        "--expect",
        choices=tuple(EXPECTED_STATES),
        default="no-part",
        help="expected terminal result; use no-part for the first empty test",
    )
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--device-id", default=DEFAULT_DEVICE_ID)
    parser.add_argument("--connection-timeout", type=positive_float, default=3.0)
    parser.add_argument("--operation-timeout", type=positive_float, default=10.0)
    parser.add_argument("--poll-interval", type=positive_float, default=0.1)
    return parser.parse_args(argv)


def state_name(value) -> str:
    if isinstance(value, int):
        return STATE_NAMES.get(value, "UNKNOWN")
    return "INVALID_TYPE"


def read_status(proxy, device_id: str) -> tuple[object, object]:
    return proxy.GetState(device_id), proxy.GetPos(device_id)


def run(cli: argparse.Namespace) -> bool:
    transport = TimeoutTransport(cli.connection_timeout)
    with xmlrpc.client.ServerProxy(
        cli.url,
        transport=transport,
        allow_none=True,
        use_builtin_types=True,
    ) as proxy:
        state, position = read_status(proxy, cli.device_id)
        print(
            f"State before grip: {state!r} ({state_name(state)}), "
            f"position={position!r}"
        )
        if not isinstance(state, int):
            print("GetState returned an unexpected non-integer value.", file=sys.stderr)
            return False
        if state == -1:
            print("Gripper reports COMMUNICATION_FAULT.", file=sys.stderr)
            return False
        if state == 0:
            print(
                "Grip blocked: the gripper is NOT_REFERENCED.",
                file=sys.stderr,
            )
            return False
        if not cli.execute:
            print("GRIP NOT SENT: add --execute only after a safety check.")
            return True
        if state != 2:
            print(
                "Grip blocked: the first test must start in RELEASED(2). Run "
                "weiss_gripper_release first.",
                file=sys.stderr,
            )
            return False

        print(
            f"Calling Grip exactly once with grip index {cli.index}; "
            f"expected result={cli.expect}. The fingers may close now."
        )
        result = proxy.Grip(cli.device_id, cli.index)
        print(format_result("Grip", result))

        deadline = time.monotonic() + cli.operation_timeout
        previous = None
        terminal_state = None
        while time.monotonic() < deadline:
            state, position = read_status(proxy, cli.device_id)
            if not isinstance(state, int):
                print(f"Unexpected GetState value: {state!r}", file=sys.stderr)
                return False
            snapshot = (state, position)
            if snapshot != previous:
                print(
                    f"Grip polling: state={state} ({state_name(state)}), "
                    f"position={position!r}"
                )
                previous = snapshot
            if state == -1:
                print("Communication fault reported during grip.", file=sys.stderr)
                return False
            if state == 0:
                print("Gripper lost its reference during grip.", file=sys.stderr)
                return False
            if state in (4, 8):
                terminal_state = state
                break
            time.sleep(cli.poll_interval)

        if terminal_state is None:
            print(
                f"Grip timed out before NO_PART or HOLDING; last state={state!r} "
                f"({state_name(state)}), position={position!r}.",
                file=sys.stderr,
            )
            return False

        expected = EXPECTED_STATES[cli.expect]
        if terminal_state != expected:
            print(
                f"Grip completed with {state_name(terminal_state)}, but "
                f"{state_name(expected)} was expected.",
                file=sys.stderr,
            )
            return False
        print(
            f"Grip test completed with expected result "
            f"{state_name(terminal_state)} at position={position!r}."
        )
        return True


def main(args=None) -> None:
    cli = parse_args(sys.argv[1:] if args is None else args)
    print("WEISS GRIPKIT grip test")
    print(f"  URL: {cli.url}")
    print(f"  device_id: {cli.device_id!r}")
    print(f"  grip_index: {cli.index}")
    print(f"  expected_result: {cli.expect}")
    print(f"  execute: {cli.execute}")
    try:
        success = run(cli)
    except xmlrpc.client.Fault as exc:
        print(
            f"XML-RPC fault: code={exc.faultCode}, message={exc.faultString}",
            file=sys.stderr,
        )
        success = False
    except xmlrpc.client.ProtocolError as exc:
        print(
            f"XML-RPC HTTP error: status={exc.errcode}, message={exc.errmsg}",
            file=sys.stderr,
        )
        success = False
    except (socket.timeout, TimeoutError) as exc:
        print(f"XML-RPC connection timed out: {exc}", file=sys.stderr)
        success = False
    except (ConnectionError, OSError, xmlrpc.client.Error) as exc:
        print(
            f"XML-RPC communication failed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        success = False
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

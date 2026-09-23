#!/usr/bin/env python3
"""Eye-in-Hand pick-and-place scenario, implemented one safe stage at a time.

Current stages: HOME joint goal, then a seeded IK observation pose.
Future stages will add camera calibration, grasp-pose input, TF conversion,
approach/retreat, and gripper operation after each stage is independently
verified on the real system.
"""

import argparse
import math
import sys
import time
import subprocess

import rclpy
from geometry_msgs.msg import PointStamped

from ur3_moveit_examples.core.ur3_moveit_controller import (
    UR3MoveItController,
    UR_JOINT_NAMES,
)


# HOME taught on the real UR3. Joint order follows UR_JOINT_NAMES.
HOME_JOINTS = (
    -0.117444,
    -0.487289,
    -1.822901,
    -0.814538,
    1.523553,
    0.954045,
)

# Measured base_link -> tool0 pose at HOME.
# Order: x, y, z, qx, qy, qz, qw.
HOME_TCP_POSE = (
    0.003780,
    0.115255,
    0.510110,
    0.683231,
    0.158719,
    -0.660302,
    -0.268337,
)

# Taught scissors observation configuration: IK search seed, not a motion goal.
OBSERVATION_JOINT_SEED = (
    1.580786, -1.751029, -1.695628, -1.270584, 1.622455, 0.954812,
)
# Target base_link -> tool0: metres, quaternion xyzw.
OBSERVATION_TCP_POSE = (
    -0.102946, -0.333081, 0.244248,
    0.454060, 0.890464, 0.016411, 0.025199,
)


def positive(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return parsed


def unit_interval(value: str) -> float:
    parsed = positive(value)
    if parsed > 1.0:
        raise argparse.ArgumentTypeError("must be in the interval (0, 1]")
    return parsed


def nonnegative(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0:
        raise argparse.ArgumentTypeError('must be finite and nonnegative')
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plan (default) or execute HOME then observation in the Eye-in-Hand "
            "pick-and-place scenario through MoveIt 2."
        )
    )
    parser.add_argument(
        "--stage", choices=("sequence", "home", "observe", "approach", "full"), default="sequence",
        help="sequence: HOME then observation; home/observe: only that stage. "
        "Plan-only sequence checks current -> HOME only.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="execute after planning and safety inspection; default is plan-only",
    )
    parser.add_argument("--velocity-scaling", type=unit_interval, default=0.03)
    parser.add_argument("--acceleration-scaling", type=unit_interval, default=0.03)
    parser.add_argument("--max-joint-travel", type=positive, default=2.10)
    parser.add_argument("--home-max-joint-travel", type=positive, default=2.15,
                        help="HOME-only joint travel limit in rad (default: 2.15)")
    parser.add_argument("--planning-time", type=positive, default=10.0)
    parser.add_argument("--planning-attempts", type=int, default=10)
    parser.add_argument("--verify-joint-tolerance", type=positive, default=0.01)
    parser.add_argument("--verify-timeout", type=positive, default=5.0)
    parser.add_argument("--server-timeout", type=positive, default=10.0)
    parser.add_argument("--group", default="ur_manipulator")
    parser.add_argument("--approach-height", type=positive,
                        help="required for approach: tool0 height above clicked point in base Z, metres")
    parser.add_argument("--candidate-timeout", type=nonnegative, default=60.0,
                        help="seconds to wait; 0 means unlimited until Ctrl+C")
    parser.add_argument("--candidate-source", choices=("manual", "auto"), default="manual")
    parser.add_argument("--confidence", type=unit_interval, default=0.4)
    parser.add_argument("--settle-time", type=positive, default=1.0)
    parser.add_argument("--no-live-view", action="store_true",
                        help="disable the continuous raw camera viewer for full stage")
    args = parser.parse_args(argv)
    if args.planning_attempts < 1:
        parser.error("--planning-attempts must be at least 1")
    if args.stage in ("approach", "full") and (
            args.approach_height is None or args.approach_height < 0.10):
        parser.error("approach requires --approach-height >= 0.10 m; clearance is not guaranteed")
    if args.stage == "full":
        args.candidate_source = "auto"
    return args


def candidate_target(msg, now_ns, started_ns, height):
    stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    xyz = (msg.point.x, msg.point.y, msg.point.z)
    if msg.header.frame_id != "base_link" or not all(map(math.isfinite, xyz)):
        raise ValueError("candidate must be finite and in base_link")
    if stamp <= started_ns or not 0 <= (now_ns-stamp)*1e-9 <= 15.0:
        raise ValueError("use a NEW snapshot after starting approach (maximum age 15 s)")
    return (*xyz[:2], xyz[2] + height, *OBSERVATION_TCP_POSE[3:])


def approach_candidate(robot, cli, detector=None):
    """One-shot operator point; never subscribe continuously during execution."""
    started = robot.get_clock().now().nanoseconds
    targets = []

    def receive(msg):
        if targets:
            return
        try:
            targets.append(candidate_target(
                msg, robot.get_clock().now().nanoseconds, started, cli.approach_height))
        except ValueError as exc:
            robot.get_logger().warn(f"Candidate rejected: {exc}")

    topic = ("/scissors/auto_preview_point" if cli.candidate_source == "auto"
             else "/scissors/candidate_point")
    subscription = robot.create_subscription(PointStamped, topic, receive, 1)
    robot.get_logger().warn(
        "Keep robot/object stopped. "
        + ("Automatic point input enabled. " if cli.candidate_source == "auto"
           else "In scissors_position press R, S, then click ONCE. ") +
        "Waiting for a new point; first valid point is frozen. "
        f"tool0 clearance above point={cli.approach_height:.3f} m. "
        "This is NOT guaranteed camera/table clearance.")
    deadline = (time.monotonic() + cli.candidate_timeout
                if cli.candidate_timeout else math.inf)
    try:
        while rclpy.ok() and not targets and time.monotonic() < deadline:
            if detector is not None and detector.poll() is not None:
                robot.get_logger().error('Automatic detector exited; scenario stopped.')
                return False
            rclpy.spin_once(robot, timeout_sec=0.1)
    finally:
        robot.destroy_subscription(subscription)
    if not targets:
        robot.get_logger().error("No valid new candidate received; no motion.")
        return False
    target = targets[0]
    if detector is not None:
        stop_detector(detector)
    robot.get_logger().warn(
        f"Frozen approach base_link -> tool0: {target}; "
        "orientation is taught observation orientation. IK seed = fresh current joints. "
        "No descent, grip, tracking, or return HOME.")
    # move_to_pose reads fresh joints and uses them as IK seed when none is supplied.
    return robot.move_to_pose(*target, execute=cli.execute)


def stop_detector(detector):
    if detector.poll() is None:
        detector.terminate()
        try:
            detector.wait(timeout=3)
        except subprocess.TimeoutExpired:
            detector.kill()
            detector.wait(timeout=3)


def automatic_approach(robot, cli):
    # Spin while stationary so joint/TF subscriptions continue to receive data.
    deadline = time.monotonic() + cli.settle_time
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(robot, timeout_sec=0.1)
    if not rclpy.ok():
        return False
    robot.get_logger().warn('Starting automatic detector after observation arrival. '
                            'First valid target will trigger planning/execution; no extra prompt.')
    # Launch Python directly, without a shell or a ros2-run wrapper to leave behind.
    detector = subprocess.Popen([
        sys.executable, '-c',
        'from ur3_moveit_examples.vision.scissors_position import main; main()',
        '--auto', '--no-window', '--auto-policy', 'first-valid', '--confidence', str(cli.confidence),
    ])
    try:
        return approach_candidate(robot, cli, detector)
    finally:
        stop_detector(detector)


def log_home(robot: UR3MoveItController, execute: bool) -> None:
    mode = "PLAN + EXECUTE" if execute else "PLAN ONLY"
    robot.get_logger().warn(f"{mode}: Eye-in-Hand scenario HOME stage")
    for name, position in zip(UR_JOINT_NAMES, HOME_JOINTS):
        robot.get_logger().info(
            f"  {name}: {position:.6f} rad ({math.degrees(position):.3f} deg)"
        )
    x, y, z, qx, qy, qz, qw = HOME_TCP_POSE
    robot.get_logger().info(
        "  taught HOME TCP (base_link -> tool0): "
        f"position=({x:.6f}, {y:.6f}, {z:.6f}), "
        f"quaternion=({qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f})"
    )


def move_home(robot, cli):
    previous_limit = robot.max_joint_travel
    try:
        robot.max_joint_travel = cli.home_max_joint_travel
        robot.get_logger().warn(
            f"HOME-only joint travel limit: {cli.home_max_joint_travel:.3f} rad; "
            f"other stages remain at {previous_limit:.3f} rad")
        return robot.move_to_joint(list(HOME_JOINTS), execute=cli.execute)
    finally:
        robot.max_joint_travel = previous_limit


def run_stages(robot: UR3MoveItController, cli: argparse.Namespace) -> bool:
    if cli.stage == "approach":
        return approach_candidate(robot, cli)
    if cli.stage in ("sequence", "home", "full"):
        log_home(robot, cli.execute)
        if not move_home(robot, cli):
            return False
        if cli.stage == "home":
            return True
        if not cli.execute:
            robot.get_logger().warn(
                "PLAN ONLY: checked current -> HOME only. No observation plan "
                "or motion. At HOME, use --stage observe to preview the IK leg."
            )
            return True

    robot.get_logger().warn(
        "Observation stage: seeded IK to the taught base_link -> tool0 pose. "
        "No YOLO, gripper operation, or automatic return HOME."
    )
    reached = robot.move_to_pose(
        *OBSERVATION_TCP_POSE,
        execute=cli.execute,
        ik_seed_positions=OBSERVATION_JOINT_SEED,
    )
    if not reached:
        return False
    if cli.stage == "full":
        return automatic_approach(robot, cli)
    return True


def main(args=None) -> None:
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    cli = parse_args(rclpy.utilities.remove_ros_args(raw_args)[1:])
    rclpy.init(args=args)
    robot = UR3MoveItController(
        node_name="ur3_eye_in_hand_pick_and_place",
        group=cli.group,
        velocity_scaling=cli.velocity_scaling,
        acceleration_scaling=cli.acceleration_scaling,
        planning_time=cli.planning_time,
        planning_attempts=cli.planning_attempts,
        max_joint_travel=cli.max_joint_travel,
        verify_joint_tolerance=cli.verify_joint_tolerance,
        verify_timeout=cli.verify_timeout,
        server_timeout=cli.server_timeout,
    )

    success = False
    viewer = None
    try:
        if cli.stage == 'full' and not cli.no_live_view:
            viewer = subprocess.Popen([
                sys.executable, '-c',
                'from ur3_moveit_examples.vision.camera_live import main; main()',
            ])
        success = run_stages(robot, cli)
        if success and viewer is not None and viewer.poll() is None:
            robot.get_logger().info('Scenario complete; no more motion. Live view remains open. '
                                    'Press q/ESC in live view or Ctrl+C here to finish.')
            while rclpy.ok() and viewer.poll() is None:
                rclpy.spin_once(robot, timeout_sec=0.1)
    except KeyboardInterrupt:
        robot.get_logger().warn("Interrupted by user.")
    except Exception as exc:
        robot.get_logger().error(
            f"Scenario stopped: {type(exc).__name__}: {exc}"
        )
    finally:
        if viewer is not None:
            stop_detector(viewer)
        robot.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Eye-in-Hand pick-and-place scenario, implemented one safe stage at a time.

Current stage: plan or execute a move to the taught HOME joint position.
Future stages will add camera calibration, grasp-pose input, TF conversion,
approach/retreat, and gripper operation after each stage is independently
verified on the real system.
"""

import argparse
import math
import sys

import rclpy

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


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plan (default) or execute the HOME stage of the Eye-in-Hand "
            "pick-and-place scenario through MoveIt 2."
        )
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="execute after planning and safety inspection; default is plan-only",
    )
    parser.add_argument("--velocity-scaling", type=unit_interval, default=0.03)
    parser.add_argument("--acceleration-scaling", type=unit_interval, default=0.03)
    parser.add_argument("--max-joint-travel", type=positive, default=2.10)
    parser.add_argument("--planning-time", type=positive, default=10.0)
    parser.add_argument("--planning-attempts", type=int, default=10)
    parser.add_argument("--verify-joint-tolerance", type=positive, default=0.01)
    parser.add_argument("--verify-timeout", type=positive, default=5.0)
    parser.add_argument("--server-timeout", type=positive, default=10.0)
    parser.add_argument("--group", default="ur_manipulator")
    args = parser.parse_args(argv)
    if args.planning_attempts < 1:
        parser.error("--planning-attempts must be at least 1")
    return args


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
    try:
        log_home(robot, cli.execute)
        success = robot.move_to_joint(list(HOME_JOINTS), execute=cli.execute)
    except KeyboardInterrupt:
        robot.get_logger().warn("Interrupted by user.")
    except Exception as exc:
        robot.get_logger().error(
            f"HOME stage failed: {type(exc).__name__}: {exc}"
        )
    finally:
        robot.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Joint-taught pick-and-place demo, currently implementing the home stage."""

import argparse
import math
import sys

import rclpy

from ur3_moveit_examples.core.ur3_moveit_controller import (
    UR3MoveItController,
    UR_JOINT_NAMES,
)
from ur3_moveit_examples.pick_place.pick_and_place_points import HOME_JOINTS


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
            "Plan (default) or execute a MoveIt joint-space motion to the "
            "taught UR3 pick-and-place home position."
        )
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--velocity-scaling", type=unit_interval, default=0.03)
    parser.add_argument("--acceleration-scaling", type=unit_interval, default=0.03)
    parser.add_argument("--max-joint-travel", type=positive, default=0.30)
    parser.add_argument("--planning-time", type=positive, default=5.0)
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--verify-joint-tolerance", type=positive, default=0.01)
    parser.add_argument("--verify-timeout", type=positive, default=3.0)
    parser.add_argument("--server-timeout", type=positive, default=10.0)
    parser.add_argument("--group", default="ur_manipulator")
    args = parser.parse_args(argv)
    if args.planning_attempts < 1:
        parser.error("--planning-attempts must be at least 1")
    return args


def main(args=None) -> None:
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    cli = parse_args(rclpy.utilities.remove_ros_args(raw_args)[1:])
    rclpy.init(args=args)
    robot = UR3MoveItController(
        node_name="ur3_pick_and_place_fk_demo",
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
        mode = "PLAN FOR EXECUTION" if cli.execute else "PLAN ONLY"
        robot.get_logger().warn(
            f"{mode}: moving to the taught pick-and-place HOME joint position."
        )
        for name, value in zip(UR_JOINT_NAMES, HOME_JOINTS):
            robot.get_logger().info(f"  {name}: {value:.6f} rad")
        success = robot.move_to_joint(list(HOME_JOINTS), execute=cli.execute)
        if success:
            robot.get_logger().info(
                "HOME stage completed. Pick and place taught positions are not "
                "implemented yet."
            )
    except KeyboardInterrupt:
        robot.get_logger().warn("Interrupted by user.")
        success = False
    except Exception as exc:
        robot.get_logger().error(f"Unexpected error: {type(exc).__name__}: {exc}")
        success = False
    finally:
        robot.destroy_node()
        rclpy.shutdown()
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""CLI wrapper for tool-frame relative TCP translation."""

import argparse
import math
import sys

import rclpy

from ur3_moveit_examples.ur3_moveit_controller import UR3MoveItController


def finite_number(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError("must be a finite number")
    return parsed


def positive(value: str) -> float:
    parsed = finite_number(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def unit_interval(value: str) -> float:
    parsed = positive(value)
    if parsed > 1.0:
        raise argparse.ArgumentTypeError("must be in the interval (0, 1]")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plan (default) or execute a translation expressed in the current "
            "tool0 coordinate frame. Units are metres."
        )
    )
    parser.add_argument("dx", type=finite_number)
    parser.add_argument("dy", type=finite_number)
    parser.add_argument("dz", type=finite_number)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--velocity-scaling", type=unit_interval, default=0.05)
    parser.add_argument("--acceleration-scaling", type=unit_interval, default=0.05)
    parser.add_argument("--max-joint-travel", type=positive, default=0.15)
    parser.add_argument("--planning-time", type=positive, default=5.0)
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--verify-position-tolerance", type=positive, default=0.003)
    parser.add_argument("--verify-orientation-tolerance", type=positive, default=0.02)
    parser.add_argument("--verify-timeout", type=positive, default=3.0)
    parser.add_argument("--server-timeout", type=positive, default=10.0)
    parser.add_argument("--group", default="ur_manipulator")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--ee-link", default="tool0")
    args = parser.parse_args(argv)
    if args.planning_attempts < 1:
        parser.error("--planning-attempts must be at least 1")
    if math.sqrt(args.dx**2 + args.dy**2 + args.dz**2) < 1.0e-9:
        parser.error("relative translation must be non-zero")
    return args


def main(args=None) -> None:
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    cli = parse_args(rclpy.utilities.remove_ros_args(raw_args)[1:])
    rclpy.init(args=args)
    robot = UR3MoveItController(
        node_name="ur3_move_relative_tool_once",
        group=cli.group,
        base_frame=cli.frame_id,
        ee_link=cli.ee_link,
        velocity_scaling=cli.velocity_scaling,
        acceleration_scaling=cli.acceleration_scaling,
        planning_time=cli.planning_time,
        planning_attempts=cli.planning_attempts,
        max_joint_travel=cli.max_joint_travel,
        verify_position_tolerance=cli.verify_position_tolerance,
        verify_orientation_tolerance=cli.verify_orientation_tolerance,
        verify_timeout=cli.verify_timeout,
        server_timeout=cli.server_timeout,
    )
    try:
        success = robot.move_relative_tool(
            cli.dx, cli.dy, cli.dz, execute=cli.execute
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

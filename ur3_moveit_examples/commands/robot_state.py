#!/usr/bin/env python3
"""CLI wrapper for UR3MoveItController state accessors."""

import argparse
import math
import sys

import rclpy

from ur3_moveit_examples.core.ur3_moveit_controller import (
    UR3MoveItController,
    UR_JOINT_NAMES,
)


def positive(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Print the current UR3 joint positions and TCP pose."
    )
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--ee-frame", default="tool0")
    parser.add_argument("--timeout", type=positive, default=10.0)
    return parser.parse_args(argv)


def main(args=None) -> None:
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    cli = parse_args(rclpy.utilities.remove_ros_args(raw_args)[1:])
    rclpy.init(args=args)
    robot = UR3MoveItController(
        node_name="ur3_robot_state",
        base_frame=cli.base_frame,
        ee_link=cli.ee_frame,
        server_timeout=cli.timeout,
    )
    try:
        joints = robot.get_current_joint_positions(cli.timeout)
        pose = robot.get_current_pose(cli.timeout)
        success = joints is not None and pose is not None
        if success:
            print("\nCurrent joint positions:")
            for name in UR_JOINT_NAMES:
                radians = joints[name]
                print(
                    f"  {name:<21} {radians: .6f} rad "
                    f"({math.degrees(radians): .3f} deg)"
                )
            print(
                f"\nCurrent TCP pose ({cli.base_frame} -> {cli.ee_frame}):"
            )
            print(
                "  position [m] : "
                f"x={pose.pose.position.x:.6f}, "
                f"y={pose.pose.position.y:.6f}, z={pose.pose.position.z:.6f}"
            )
            print(
                "  quaternion  : "
                f"x={pose.pose.orientation.x:.6f}, "
                f"y={pose.pose.orientation.y:.6f}, "
                f"z={pose.pose.orientation.z:.6f}, "
                f"w={pose.pose.orientation.w:.6f}"
            )
    except KeyboardInterrupt:
        robot.get_logger().warn("Interrupted by user.")
        success = False
    finally:
        robot.destroy_node()
        rclpy.shutdown()
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

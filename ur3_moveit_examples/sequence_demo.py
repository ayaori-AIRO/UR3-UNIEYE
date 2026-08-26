#!/usr/bin/env python3
"""Safely demonstrate two sequential TCP motions with UR3MoveItController."""

import argparse
import math
import sys

import rclpy

from ur3_moveit_examples.ur3_moveit_controller import UR3MoveItController


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
            "Plan a TCP Z-axis lift, or execute the lift and return sequence "
            "through MoveIt 2."
        )
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--lift-distance", type=positive, default=0.01)
    parser.add_argument("--velocity-scaling", type=unit_interval, default=0.05)
    parser.add_argument("--acceleration-scaling", type=unit_interval, default=0.05)
    parser.add_argument("--max-joint-travel", type=positive, default=0.15)
    parser.add_argument("--planning-time", type=positive, default=5.0)
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--server-timeout", type=positive, default=10.0)
    parser.add_argument("--group", default="ur_manipulator")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--ee-link", default="tool0")
    args = parser.parse_args(argv)
    if args.planning_attempts < 1:
        parser.error("--planning-attempts must be at least 1")
    return args


def move_to_pose(robot: UR3MoveItController, pose, *, execute: bool) -> bool:
    position = pose.position
    orientation = pose.orientation
    return robot.move_to_pose(
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
        execute=execute,
    )


def main(args=None) -> None:
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    cli = parse_args(rclpy.utilities.remove_ros_args(raw_args)[1:])
    rclpy.init(args=args)
    robot = UR3MoveItController(
        node_name="ur3_sequence_demo",
        group=cli.group,
        base_frame=cli.frame_id,
        ee_link=cli.ee_link,
        velocity_scaling=cli.velocity_scaling,
        acceleration_scaling=cli.acceleration_scaling,
        planning_time=cli.planning_time,
        planning_attempts=cli.planning_attempts,
        max_joint_travel=cli.max_joint_travel,
        server_timeout=cli.server_timeout,
    )
    success = False
    try:
        start = robot.get_current_pose()
        if start is None:
            return_code = 1
        else:
            robot.get_logger().info(
                "Sequence start TCP: "
                f"x={start.pose.position.x:.6f}, "
                f"y={start.pose.position.y:.6f}, "
                f"z={start.pose.position.z:.6f}"
            )
            lifted = type(start.pose)()
            lifted.position.x = start.pose.position.x
            lifted.position.y = start.pose.position.y
            lifted.position.z = start.pose.position.z + cli.lift_distance
            lifted.orientation = start.pose.orientation

            robot.get_logger().warn(
                f"Step 1: TCP Z lift by {cli.lift_distance:.6f} m"
            )
            success = move_to_pose(robot, lifted, execute=cli.execute)

            if success and cli.execute:
                robot.get_logger().warn("Step 2: return to the recorded start TCP pose")
                success = move_to_pose(robot, start.pose, execute=True)
            elif success:
                robot.get_logger().info(
                    "Plan-only succeeded for the lift. The return step is skipped "
                    "because the robot did not actually move."
                )
            return_code = 0 if success else 1
    except KeyboardInterrupt:
        robot.get_logger().warn("Interrupted by user; no further sequence step will run.")
        return_code = 1
    except Exception as exc:
        robot.get_logger().error(f"Unexpected error: {type(exc).__name__}: {exc}")
        return_code = 1
    finally:
        robot.destroy_node()
        rclpy.shutdown()
    if return_code:
        raise SystemExit(return_code)


if __name__ == "__main__":
    main()

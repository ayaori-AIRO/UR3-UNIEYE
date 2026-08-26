#!/usr/bin/env python3
"""Empty-space Cartesian approach and retreat scenario for a real UR3."""

import argparse
import math
import sys
import time

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


def non_negative(value: str) -> float:
    parsed = finite_number(value)
    if parsed < 0.0:
        raise argparse.ArgumentTypeError("must be zero or greater")
    return parsed


def unit_interval(value: str) -> float:
    parsed = positive(value)
    if parsed > 1.0:
        raise argparse.ArgumentTypeError("must be in the interval (0, 1]")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plan an approach, or execute a Cartesian tool-Z approach and "
            "retreat scenario in empty space."
        )
    )
    parser.add_argument(
        "--distance",
        type=finite_number,
        default=0.005,
        help="signed tool-Z approach distance in metres (default: +0.005)",
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dwell-time", type=non_negative, default=1.0)
    parser.add_argument("--max-step", type=positive, default=0.001)
    parser.add_argument("--jump-threshold", type=non_negative, default=2.0)
    parser.add_argument("--minimum-fraction", type=unit_interval, default=0.999)
    parser.add_argument("--velocity-scaling", type=unit_interval, default=0.05)
    parser.add_argument("--acceleration-scaling", type=unit_interval, default=0.05)
    parser.add_argument("--max-joint-travel", type=positive, default=0.15)
    parser.add_argument("--verify-position-tolerance", type=positive, default=0.003)
    parser.add_argument("--verify-orientation-tolerance", type=positive, default=0.02)
    parser.add_argument("--verify-timeout", type=positive, default=3.0)
    parser.add_argument("--server-timeout", type=positive, default=10.0)
    parser.add_argument("--group", default="ur_manipulator")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--ee-link", default="tool0")
    args = parser.parse_args(argv)
    if abs(args.distance) < 1.0e-9:
        parser.error("--distance must be non-zero")
    return args


def pose_errors(actual, target) -> tuple[float, float]:
    position_error = math.sqrt(
        (actual.position.x - target.position.x) ** 2
        + (actual.position.y - target.position.y) ** 2
        + (actual.position.z - target.position.z) ** 2
    )
    dot = abs(
        actual.orientation.x * target.orientation.x
        + actual.orientation.y * target.orientation.y
        + actual.orientation.z * target.orientation.z
        + actual.orientation.w * target.orientation.w
    )
    orientation_error = 2.0 * math.acos(max(0.0, min(1.0, dot)))
    return position_error, orientation_error


def cartesian_tool_z(
    robot: UR3MoveItController, distance: float, cli, *, execute: bool
) -> bool:
    return robot.move_cartesian_relative_tool(
        0.0,
        0.0,
        distance,
        execute=execute,
        max_step=cli.max_step,
        jump_threshold=cli.jump_threshold,
        minimum_fraction=cli.minimum_fraction,
    )


def cartesian_to_pose(
    robot: UR3MoveItController, pose, cli, *, execute: bool
) -> bool:
    return robot.move_cartesian_to_pose(
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
        execute=execute,
        max_step=cli.max_step,
        jump_threshold=cli.jump_threshold,
        minimum_fraction=cli.minimum_fraction,
    )


def main(args=None) -> None:
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    cli = parse_args(rclpy.utilities.remove_ros_args(raw_args)[1:])
    rclpy.init(args=args)
    robot = UR3MoveItController(
        node_name="ur3_approach_retreat_demo",
        group=cli.group,
        base_frame=cli.frame_id,
        ee_link=cli.ee_link,
        velocity_scaling=cli.velocity_scaling,
        acceleration_scaling=cli.acceleration_scaling,
        max_joint_travel=cli.max_joint_travel,
        verify_position_tolerance=cli.verify_position_tolerance,
        verify_orientation_tolerance=cli.verify_orientation_tolerance,
        verify_timeout=cli.verify_timeout,
        server_timeout=cli.server_timeout,
    )
    success = False
    try:
        start = robot.get_current_pose()
        if start is None:
            success = False
        else:
            robot.get_logger().info(
                "Scenario start TCP: "
                f"x={start.pose.position.x:.6f}, "
                f"y={start.pose.position.y:.6f}, "
                f"z={start.pose.position.z:.6f}"
            )
            robot.get_logger().warn(
                f"Step 1/2: Cartesian approach along tool Z by "
                f"{cli.distance:.6f} m"
            )
            success = cartesian_tool_z(
                robot, cli.distance, cli, execute=cli.execute
            )

            if success and cli.execute:
                robot.get_logger().info(
                    f"Approach completed; holding for {cli.dwell_time:.3f} s."
                )
                if cli.dwell_time > 0.0:
                    time.sleep(cli.dwell_time)
                robot.get_logger().warn(
                    "Step 2/2: Cartesian retreat to the recorded start TCP pose"
                )
                success = cartesian_to_pose(
                    robot, start.pose, cli, execute=True
                )
                if success:
                    final = robot.get_current_pose()
                    if final is None:
                        success = False
                    else:
                        position_error, orientation_error = pose_errors(
                            final.pose, start.pose
                        )
                        success = (
                            position_error <= cli.verify_position_tolerance
                            and orientation_error
                            <= cli.verify_orientation_tolerance
                        )
                        log = (
                            robot.get_logger().info
                            if success
                            else robot.get_logger().error
                        )
                        log(
                            "Scenario return verification "
                            f"{'succeeded' if success else 'failed'}: "
                            f"position_error={position_error:.6f} m, "
                            f"orientation_error={orientation_error:.6f} rad"
                        )
            elif success:
                robot.get_logger().info(
                    "Plan-only succeeded for the approach. The retreat step is "
                    "skipped because the robot did not actually move."
                )
    except KeyboardInterrupt:
        robot.get_logger().warn(
            "Interrupted by user; no further scenario step will run."
        )
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

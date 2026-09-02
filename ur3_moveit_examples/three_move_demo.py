#!/usr/bin/env python3
"""Simple three-motion MoveIt demonstration for a real UR3."""

import argparse
import math
import sys
import time

import rclpy

from ur3_moveit_examples.ur3_moveit_controller import (
    UR3MoveItController,
    UR_JOINT_NAMES,
)


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
            "Plan the first motion (default), or execute +offset, -offset, "
            "and return-to-start joint motions through MoveIt 2."
        )
    )
    parser.add_argument(
        "--joint",
        choices=UR_JOINT_NAMES,
        default="shoulder_pan_joint",
    )
    parser.add_argument("--offset", type=positive, default=0.05)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dwell-time", type=non_negative, default=0.5)
    parser.add_argument("--velocity-scaling", type=unit_interval, default=0.05)
    parser.add_argument("--acceleration-scaling", type=unit_interval, default=0.05)
    parser.add_argument("--max-joint-travel", type=positive, default=0.15)
    parser.add_argument("--planning-time", type=positive, default=5.0)
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--verify-joint-tolerance", type=positive, default=0.01)
    parser.add_argument("--verify-timeout", type=positive, default=3.0)
    parser.add_argument("--server-timeout", type=positive, default=10.0)
    parser.add_argument("--group", default="ur_manipulator")
    args = parser.parse_args(argv)
    if args.planning_attempts < 1:
        parser.error("--planning-attempts must be at least 1")
    if 2.0 * args.offset > args.max_joint_travel:
        parser.error(
            "twice --offset must not exceed --max-joint-travel because "
            "the second motion travels from +offset to -offset"
        )
    return args


def make_targets(
    start: list[float], joint_name: str, offset: float
) -> tuple[list[float], list[float], list[float]]:
    index = UR_JOINT_NAMES.index(joint_name)
    positive = list(start)
    negative = list(start)
    positive[index] += offset
    negative[index] -= offset
    return positive, negative, list(start)


def main(args=None) -> None:
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    cli = parse_args(rclpy.utilities.remove_ros_args(raw_args)[1:])
    rclpy.init(args=args)
    robot = UR3MoveItController(
        node_name="ur3_three_move_demo",
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
        current = robot.get_current_joint_positions()
        if current is not None:
            start = [current[name] for name in UR_JOINT_NAMES]
            targets = make_targets(start, cli.joint, cli.offset)
            robot.get_logger().warn(
                f"Three-move demo: joint={cli.joint}, offset={cli.offset:.6f} "
                f"rad ({math.degrees(cli.offset):.3f} deg), execute={cli.execute}"
            )

            if not cli.execute:
                robot.get_logger().warn(
                    "PLAN ONLY: checking step 1/3 (+offset). Steps 2 and 3 "
                    "are planned from the actual state only during execution."
                )
                success = robot.move_to_joint(targets[0], execute=False)
            else:
                labels = (
                    "Step 1/3: +offset",
                    "Step 2/3: -offset",
                    "Step 3/3: return to recorded start",
                )
                success = True
                for step, (label, target) in enumerate(zip(labels, targets)):
                    robot.get_logger().warn(label)
                    if not robot.move_to_joint(target, execute=True):
                        success = False
                        robot.get_logger().error(
                            "Three-move demo stopped because the current step failed."
                        )
                        break
                    if step < 2 and cli.dwell_time > 0.0:
                        robot.get_logger().info(
                            f"Holding for {cli.dwell_time:.3f} s before next step."
                        )
                        time.sleep(cli.dwell_time)
                if success:
                    robot.get_logger().info(
                        "Three-move demo completed and returned to the start joints."
                    )
    except KeyboardInterrupt:
        robot.get_logger().warn("Interrupted by user; no further step will run.")
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

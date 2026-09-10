#!/usr/bin/env python3
"""Coordinated whole-arm three-motion demonstration for a real UR3."""

import argparse
import math
import sys
import time

import rclpy

from ur3_moveit_examples.core.ur3_moveit_controller import (
    UR3MoveItController,
    UR_JOINT_NAMES,
)


BASE_DELTAS = {
    "shoulder_lift_joint": 0.04,
    "elbow_joint": -0.06,
    "wrist_1_joint": 0.02,
}


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
            "Plan the first coordinated pose (default), or execute pose A, "
            "pose B, and return-to-start motions through MoveIt 2."
        )
    )
    parser.add_argument("--motion-scale", type=positive, default=1.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dwell-time", type=non_negative, default=0.7)
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
    largest_span = 2.0 * max(abs(value) for value in BASE_DELTAS.values())
    if largest_span * args.motion_scale > args.max_joint_travel:
        parser.error(
            "the A-to-B joint span exceeds --max-joint-travel; reduce "
            "--motion-scale or increase the limit only after a safety review"
        )
    return args


def make_targets(
    start: list[float], motion_scale: float
) -> tuple[list[float], list[float], list[float]]:
    pose_a = list(start)
    pose_b = list(start)
    for name, delta in BASE_DELTAS.items():
        index = UR_JOINT_NAMES.index(name)
        scaled = delta * motion_scale
        pose_a[index] += scaled
        pose_b[index] -= scaled
    return pose_a, pose_b, list(start)


def main(args=None) -> None:
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    cli = parse_args(rclpy.utilities.remove_ros_args(raw_args)[1:])
    rclpy.init(args=args)
    robot = UR3MoveItController(
        node_name="ur3_arm_sweep_demo",
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
            targets = make_targets(start, cli.motion_scale)
            robot.get_logger().warn(
                f"Whole-arm sweep demo: motion_scale={cli.motion_scale:.3f}, "
                f"execute={cli.execute}"
            )
            for name, delta in BASE_DELTAS.items():
                scaled = delta * cli.motion_scale
                robot.get_logger().warn(
                    f"  {name}: pose_A_delta={scaled:+.6f} rad, "
                    f"pose_B_delta={-scaled:+.6f} rad"
                )

            if not cli.execute:
                robot.get_logger().warn(
                    "PLAN ONLY: checking pose A. Poses B and start are planned "
                    "from the actual state only during execution."
                )
                success = robot.move_to_joint(targets[0], execute=False)
            else:
                labels = (
                    "Step 1/3: coordinated pose A",
                    "Step 2/3: coordinated pose B",
                    "Step 3/3: return to recorded start",
                )
                success = True
                for step, (label, target) in enumerate(zip(labels, targets)):
                    robot.get_logger().warn(label)
                    if not robot.move_to_joint(target, execute=True):
                        success = False
                        robot.get_logger().error(
                            "Whole-arm sweep stopped because the current step failed."
                        )
                        break
                    if step < 2 and cli.dwell_time > 0.0:
                        robot.get_logger().info(
                            f"Holding for {cli.dwell_time:.3f} s before next step."
                        )
                        time.sleep(cli.dwell_time)
                if success:
                    robot.get_logger().info(
                        "Whole-arm sweep completed and returned to the start joints."
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

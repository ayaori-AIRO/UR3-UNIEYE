#!/usr/bin/env python3
"""Fixed joint-taught Point 1 -> grip -> Point 2 -> release -> home scenario."""

import argparse
import math
import socket
import sys
import time
import xmlrpc.client

import rclpy

from ur3_moveit_examples.pick_place.pick_and_place_points import (
    HOME_JOINTS,
    POINT1_JOINTS,
    POINT2_JOINTS,
)
from ur3_moveit_examples.core.ur3_moveit_controller import (
    UR3MoveItController,
    UR_JOINT_NAMES,
)
from ur3_moveit_examples.gripper.weiss_gripper import (
    HOLDING,
    NO_PART,
    WeissGripper,
    WeissGripperError,
)
from ur3_moveit_examples.gripper.weiss_gripper_read_state import (
    DEFAULT_DEVICE_ID,
    DEFAULT_URL,
)


def positive(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return parsed


def non_negative(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError("must be a non-negative finite number")
    return parsed


def unit_interval(value: str) -> float:
    parsed = positive(value)
    if parsed > 1.0:
        raise argparse.ArgumentTypeError("must be in the interval (0, 1]")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plan Point 1 (default), or execute a fixed joint-taught pick and "
            "place sequence with WEISS XML-RPC grip/release."
        )
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--allow-empty-grip",
        action="store_true",
        help=(
            "demo without a workpiece: allow the expected NO_PART result and "
            "continue to Point 2 (only valid with --execute)"
        ),
    )
    parser.add_argument("--grip-index", type=int, default=0)
    parser.add_argument("--dwell-time", type=non_negative, default=0.5)
    parser.add_argument("--velocity-scaling", type=unit_interval, default=0.03)
    parser.add_argument("--acceleration-scaling", type=unit_interval, default=0.03)
    parser.add_argument("--max-joint-travel", type=positive, default=2.10)
    parser.add_argument("--planning-time", type=positive, default=10.0)
    parser.add_argument("--planning-attempts", type=int, default=10)
    parser.add_argument("--verify-joint-tolerance", type=positive, default=0.01)
    parser.add_argument("--verify-timeout", type=positive, default=5.0)
    parser.add_argument("--server-timeout", type=positive, default=10.0)
    parser.add_argument("--gripper-url", default=DEFAULT_URL)
    parser.add_argument("--device-id", default=DEFAULT_DEVICE_ID)
    parser.add_argument("--gripper-timeout", type=positive, default=10.0)
    parser.add_argument("--group", default="ur_manipulator")
    args = parser.parse_args(argv)
    if args.grip_index < 0:
        parser.error("--grip-index must be zero or greater")
    if args.allow_empty_grip and not args.execute:
        parser.error("--allow-empty-grip requires --execute")
    if args.planning_attempts < 1:
        parser.error("--planning-attempts must be at least 1")
    taught_legs = (
        (HOME_JOINTS, POINT1_JOINTS),
        (POINT1_JOINTS, POINT2_JOINTS),
        (POINT2_JOINTS, HOME_JOINTS),
    )
    minimum_required = max(
        abs(end - start)
        for leg_start, leg_end in taught_legs
        for start, end in zip(leg_start, leg_end)
    )
    if args.max_joint_travel + 1.0e-9 < minimum_required:
        parser.error(
            f"--max-joint-travel must be at least {minimum_required:.6f} rad "
            "for the taught Point 1, Point 2, and Home spans"
        )
    return args


def log_target(robot: UR3MoveItController, name: str, target) -> None:
    robot.get_logger().info(f"{name} joint target:")
    for joint, value in zip(UR_JOINT_NAMES, target):
        robot.get_logger().info(f"  {joint}: {value:.6f} rad")


def move(
    robot: UR3MoveItController,
    label: str,
    target,
    *,
    execute: bool,
) -> bool:
    robot.get_logger().warn(label)
    log_target(robot, label, target)
    return robot.move_to_joint(list(target), execute=execute)


def main(args=None) -> None:
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    cli = parse_args(rclpy.utilities.remove_ros_args(raw_args)[1:])
    rclpy.init(args=args)
    robot = UR3MoveItController(
        node_name="ur3_fk_pick_and_place_demo",
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
        if not cli.execute:
            robot.get_logger().warn(
                "PLAN ONLY: checking current position -> Point 1. No gripper "
                "command, Point 2 motion, or Home motion will run."
            )
            success = move(
                robot,
                "Step 1/5: move to Point 1",
                POINT1_JOINTS,
                execute=False,
            )
        else:
            if cli.allow_empty_grip:
                robot.get_logger().warn(
                    "EMPTY-GRIP DEMO MODE: NO_PART(4) will be accepted. "
                    "This mode must not be used for a real workpiece."
                )
            with WeissGripper(
                url=cli.gripper_url,
                device_id=cli.device_id,
                operation_timeout=cli.gripper_timeout,
            ) as gripper:
                gripper.require_released()
                robot.get_logger().info(
                    "Gripper precondition succeeded: RELEASED(2), "
                    f"position={gripper.get_position():.3f} mm"
                )

                if not move(
                    robot,
                    "Step 1/5: move to Point 1",
                    POINT1_JOINTS,
                    execute=True,
                ):
                    raise RuntimeError("Point 1 motion failed")

                robot.get_logger().warn("Step 2/5: grip at Point 1")
                grip_state = gripper.grip(cli.grip_index)
                grip_position = gripper.get_position()
                if grip_state == NO_PART:
                    if not cli.allow_empty_grip:
                        raise WeissGripperError(
                            "Grip failed: NO_PART at "
                            f"position={grip_position:.3f} mm"
                        )
                    robot.get_logger().warn(
                        "Empty-grip demo accepted NO_PART(4), "
                        f"position={grip_position:.3f} mm"
                    )
                elif grip_state != HOLDING:
                    raise WeissGripperError(
                        f"Grip failed: unexpected state={grip_state}"
                    )
                else:
                    robot.get_logger().info(
                        "Grip succeeded: HOLDING(8), "
                        f"position={grip_position:.3f} mm"
                    )
                if cli.dwell_time > 0.0:
                    time.sleep(cli.dwell_time)

                if not move(
                    robot,
                    (
                        "Step 3/5: move to Point 2 with empty closed gripper"
                        if cli.allow_empty_grip
                        else "Step 3/5: move to Point 2 while HOLDING"
                    ),
                    POINT2_JOINTS,
                    execute=True,
                ):
                    raise RuntimeError("Point 2 motion failed while holding the part")
                state_at_point2 = gripper.get_state()
                expected_state = NO_PART if cli.allow_empty_grip else HOLDING
                if state_at_point2 != expected_state:
                    raise WeissGripperError(
                        "gripper state check failed after the Point 2 motion; "
                        f"expected={expected_state}, actual={state_at_point2}"
                    )

                robot.get_logger().warn("Step 4/5: release at Point 2")
                gripper.release(cli.grip_index)
                robot.get_logger().info(
                    "Release succeeded: RELEASED(2), "
                    f"position={gripper.get_position():.3f} mm"
                )
                if cli.dwell_time > 0.0:
                    time.sleep(cli.dwell_time)

                if not move(
                    robot,
                    "Step 5/5: return to Home",
                    HOME_JOINTS,
                    execute=True,
                ):
                    raise RuntimeError("Home motion failed")
                success = True
                robot.get_logger().info("Pick-and-place scenario completed.")
    except KeyboardInterrupt:
        robot.get_logger().error(
            "Scenario interrupted. Inspect the gripper and robot before retrying."
        )
    except (
        WeissGripperError,
        xmlrpc.client.Error,
        socket.timeout,
        TimeoutError,
        ConnectionError,
        OSError,
        RuntimeError,
    ) as exc:
        robot.get_logger().error(
            f"Scenario stopped: {type(exc).__name__}: {exc}"
        )
    finally:
        robot.destroy_node()
        rclpy.shutdown()
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

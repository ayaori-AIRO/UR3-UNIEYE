#!/usr/bin/env python3
"""Pose/IK based pick-and-place with Cartesian approach and retreat."""

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
    POINT1_POSE,
    POINT2_JOINTS,
    POINT2_POSE,
)
from ur3_moveit_examples.core.ur3_moveit_controller import UR3MoveItController
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
            "Use collision-aware IK for taught TCP poses, Cartesian vertical "
            "approach/retreat, and WEISS grip/release. Default is plan-only."
        )
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--allow-empty-grip",
        action="store_true",
        help="allow NO_PART for a no-workpiece demonstration; requires --execute",
    )
    parser.add_argument(
        "--approach-height",
        type=positive,
        default=0.05,
        help="vertical base_link +Z clearance above Point 1 and Point 2 [m]",
    )
    parser.add_argument("--max-step", type=positive, default=0.002)
    parser.add_argument("--minimum-fraction", type=unit_interval, default=0.999)
    parser.add_argument("--jump-threshold", type=non_negative, default=2.0)
    parser.add_argument("--grip-index", type=int, default=0)
    parser.add_argument("--dwell-time", type=non_negative, default=0.5)
    parser.add_argument("--velocity-scaling", type=unit_interval, default=0.03)
    parser.add_argument("--acceleration-scaling", type=unit_interval, default=0.03)
    parser.add_argument("--max-joint-travel", type=positive, default=2.10)
    parser.add_argument("--planning-time", type=positive, default=10.0)
    parser.add_argument("--planning-attempts", type=int, default=10)
    parser.add_argument("--verify-joint-tolerance", type=positive, default=0.01)
    parser.add_argument("--verify-position-tolerance", type=positive, default=0.005)
    parser.add_argument(
        "--verify-orientation-tolerance", type=positive, default=0.03
    )
    parser.add_argument("--verify-timeout", type=positive, default=5.0)
    parser.add_argument("--server-timeout", type=positive, default=10.0)
    parser.add_argument("--gripper-url", default=DEFAULT_URL)
    parser.add_argument("--device-id", default=DEFAULT_DEVICE_ID)
    parser.add_argument("--gripper-timeout", type=positive, default=10.0)
    parser.add_argument("--group", default="ur_manipulator")
    args = parser.parse_args(argv)
    if args.allow_empty_grip and not args.execute:
        parser.error("--allow-empty-grip requires --execute")
    if args.grip_index < 0:
        parser.error("--grip-index must be zero or greater")
    if args.planning_attempts < 1:
        parser.error("--planning-attempts must be at least 1")
    return args


def approach_pose(target: tuple[float, ...], height: float) -> tuple[float, ...]:
    return (target[0], target[1], target[2] + height, *target[3:])


def move_pose(
    robot: UR3MoveItController,
    label: str,
    pose: tuple[float, ...],
    ik_seed: tuple[float, ...],
    *,
    execute: bool,
) -> bool:
    robot.get_logger().warn(label)
    return robot.move_to_pose(
        *pose,
        execute=execute,
        ik_seed_positions=ik_seed,
    )


def move_cartesian(
    robot: UR3MoveItController,
    label: str,
    pose: tuple[float, ...],
    cli: argparse.Namespace,
) -> bool:
    robot.get_logger().warn(label)
    return robot.move_cartesian_to_pose(
        *pose,
        execute=True,
        max_step=cli.max_step,
        jump_threshold=cli.jump_threshold,
        minimum_fraction=cli.minimum_fraction,
    )


def require_grip_state(
    gripper: WeissGripper,
    expected: int,
    location: str,
) -> None:
    actual = gripper.get_state()
    if actual != expected:
        raise WeissGripperError(
            f"gripper state check failed {location}; expected={expected}, "
            f"actual={actual} ({gripper.state_name(actual)})"
        )


def main(args=None) -> None:
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    cli = parse_args(rclpy.utilities.remove_ros_args(raw_args)[1:])
    rclpy.init(args=args)
    robot = UR3MoveItController(
        node_name="ur3_ik_pick_and_place_demo",
        group=cli.group,
        velocity_scaling=cli.velocity_scaling,
        acceleration_scaling=cli.acceleration_scaling,
        planning_time=cli.planning_time,
        planning_attempts=cli.planning_attempts,
        max_joint_travel=cli.max_joint_travel,
        verify_joint_tolerance=cli.verify_joint_tolerance,
        verify_position_tolerance=cli.verify_position_tolerance,
        verify_orientation_tolerance=cli.verify_orientation_tolerance,
        verify_timeout=cli.verify_timeout,
        server_timeout=cli.server_timeout,
    )
    pick_approach = approach_pose(POINT1_POSE, cli.approach_height)
    place_approach = approach_pose(POINT2_POSE, cli.approach_height)
    success = False
    try:
        if not cli.execute:
            robot.get_logger().warn(
                "PLAN ONLY: checking current pose -> Point 1 approach using IK. "
                "No descent, gripper command, Point 2 motion, or Home motion."
            )
            success = move_pose(
                robot,
                "Step 1/9: IK plan to Point 1 approach",
                pick_approach,
                POINT1_JOINTS,
                execute=False,
            )
        else:
            if cli.allow_empty_grip:
                robot.get_logger().warn(
                    "EMPTY-GRIP DEMO MODE: NO_PART(4) will be accepted. "
                    "Do not use this option with a real workpiece."
                )
            with WeissGripper(
                url=cli.gripper_url,
                device_id=cli.device_id,
                operation_timeout=cli.gripper_timeout,
            ) as gripper:
                gripper.require_released()

                if not move_pose(
                    robot,
                    "Step 1/9: IK move to Point 1 approach",
                    pick_approach,
                    POINT1_JOINTS,
                    execute=True,
                ):
                    raise RuntimeError("Point 1 approach motion failed")
                if not move_cartesian(
                    robot,
                    "Step 2/9: Cartesian descent to Point 1",
                    POINT1_POSE,
                    cli,
                ):
                    raise RuntimeError("Point 1 Cartesian descent failed")

                robot.get_logger().warn("Step 3/9: grip at Point 1")
                grip_state = gripper.grip(cli.grip_index)
                grip_position = gripper.get_position()
                expected_grip_state = NO_PART if cli.allow_empty_grip else HOLDING
                if grip_state != expected_grip_state:
                    raise WeissGripperError(
                        f"Grip result mismatch: expected={expected_grip_state}, "
                        f"actual={grip_state}, position={grip_position:.3f} mm"
                    )
                robot.get_logger().info(
                    f"Grip result accepted: {gripper.state_name(grip_state)}"
                    f"({grip_state}), position={grip_position:.3f} mm"
                )
                if cli.dwell_time > 0.0:
                    time.sleep(cli.dwell_time)

                if not move_cartesian(
                    robot,
                    "Step 4/9: Cartesian retreat from Point 1",
                    pick_approach,
                    cli,
                ):
                    raise RuntimeError("Point 1 Cartesian retreat failed")
                require_grip_state(gripper, expected_grip_state, "after pick retreat")

                if not move_pose(
                    robot,
                    "Step 5/9: IK move to Point 2 approach",
                    place_approach,
                    POINT2_JOINTS,
                    execute=True,
                ):
                    raise RuntimeError("Point 2 approach motion failed")
                require_grip_state(gripper, expected_grip_state, "at Point 2 approach")

                if not move_cartesian(
                    robot,
                    "Step 6/9: Cartesian descent to Point 2",
                    POINT2_POSE,
                    cli,
                ):
                    raise RuntimeError("Point 2 Cartesian descent failed")

                robot.get_logger().warn("Step 7/9: release at Point 2")
                gripper.release(cli.grip_index)
                if cli.dwell_time > 0.0:
                    time.sleep(cli.dwell_time)

                if not move_cartesian(
                    robot,
                    "Step 8/9: Cartesian retreat from Point 2",
                    place_approach,
                    cli,
                ):
                    raise RuntimeError("Point 2 Cartesian retreat failed")

                robot.get_logger().warn(
                    "Step 9/9: return to deterministic taught Home joint pose"
                )
                if not robot.move_to_joint(list(HOME_JOINTS), execute=True):
                    raise RuntimeError("Home motion failed")
                success = True
                robot.get_logger().info("IK pick-and-place scenario completed.")
    except KeyboardInterrupt:
        robot.get_logger().error(
            "Scenario interrupted. Inspect the robot and gripper before retrying."
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

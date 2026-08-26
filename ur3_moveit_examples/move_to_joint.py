#!/usr/bin/env python3
"""Plan or execute one UR3 joint-space goal through MoveIt 2."""

import argparse
import math
import sys
import time

import rclpy
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from ur3_moveit_examples.trajectory_safety import (
    joint_excursions,
    trajectory_duration_seconds,
)


UR_JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

ERROR_NAMES = {
    value: name
    for name, value in vars(MoveItErrorCodes).items()
    if name.isupper() and isinstance(value, int)
}


class MoveToJointOnce(Node):
    """Send one collision-aware joint goal to MoveIt's MoveGroup action."""

    def __init__(self) -> None:
        super().__init__("ur3_move_to_joint_once")
        self._client = ActionClient(self, MoveGroup, "/move_action")
        self._execute_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )
        self._joint_positions: dict[str, float] | None = None
        self._joint_state_sequence = 0
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            qos_profile_sensor_data,
        )

    def _joint_state_callback(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in UR_JOINT_NAMES):
            self._joint_positions = {
                name: positions[name] for name in UR_JOINT_NAMES
            }
            self._joint_state_sequence += 1

    def run(self, args: argparse.Namespace) -> bool:
        self.get_logger().info("Waiting for MoveIt /move_action ...")
        if not self._client.wait_for_server(timeout_sec=args.server_timeout):
            self.get_logger().error(
                "MoveIt action server was not found. Is ur_moveit.launch.py running?"
            )
            return False
        if args.execute and not self._execute_client.wait_for_server(
            timeout_sec=args.server_timeout
        ):
            self.get_logger().error(
                "MoveIt /execute_trajectory action server was not found."
            )
            return False

        positions = self._positions_in_radians(args)
        goal = self._make_goal(args, positions)
        mode = "PLAN FOR EXECUTION" if args.execute else "PLAN ONLY"

        self.get_logger().warn(f"{mode}: joint target [rad]")
        for name, position in zip(UR_JOINT_NAMES, positions):
            self.get_logger().warn(
                f"  {name}: {position:.6f} rad ({math.degrees(position):.3f} deg)"
            )

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveIt rejected the goal.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error("No result received from MoveIt.")
            return False

        result = wrapped_result.result
        code = result.error_code.val
        code_name = ERROR_NAMES.get(code, f"UNKNOWN_ERROR_{code}")
        point_count = len(result.planned_trajectory.joint_trajectory.points)

        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"MoveIt failed: {code_name} ({code}); planning_time="
                f"{result.planning_time:.3f} s"
            )
            return False

        self.get_logger().info(
            f"MoveIt planning succeeded: {code_name}; planning_time="
            f"{result.planning_time:.3f} s; trajectory_points={point_count}; mode={mode}"
        )

        if not self._inspect_trajectory(result, args):
            return False
        if args.execute:
            if not self._execute_trajectory(result.planned_trajectory):
                return False
            return self._verify_joint_goal(positions, args)
        return True

    def _inspect_trajectory(self, result, args: argparse.Namespace) -> bool:
        trajectory = result.planned_trajectory
        excursions = joint_excursions(result.trajectory_start, trajectory)
        if not excursions:
            self.get_logger().error("Planned trajectory contains no joint points.")
            return False

        maximum_name = max(excursions, key=excursions.get)
        maximum = excursions[maximum_name]
        duration = trajectory_duration_seconds(trajectory)
        self.get_logger().info(
            f"Trajectory inspection: duration={duration:.3f} s, "
            f"maximum_joint_travel={maximum:.6f} rad ({maximum_name})"
        )
        for name, travel in excursions.items():
            self.get_logger().info(f"  {name}: maximum_travel={travel:.6f} rad")

        if args.execute and maximum > args.max_joint_travel:
            self.get_logger().error(
                "Execution blocked: planned joint travel exceeds the configured "
                f"limit. maximum={maximum:.6f} rad, limit="
                f"{args.max_joint_travel:.6f} rad, joint={maximum_name}"
            )
            return False
        return True

    def _execute_trajectory(self, trajectory) -> bool:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        self.get_logger().warn("Executing the exact inspected trajectory ...")

        send_future = self._execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveIt rejected the trajectory execution goal.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error("No trajectory execution result received.")
            return False

        code = wrapped_result.result.error_code.val
        code_name = ERROR_NAMES.get(code, f"UNKNOWN_ERROR_{code}")
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Trajectory execution failed: {code_name} ({code})"
            )
            return False

        self.get_logger().info(f"Trajectory execution succeeded: {code_name}")
        return True

    def _verify_joint_goal(
        self, target_positions: list[float], args: argparse.Namespace
    ) -> bool:
        self.get_logger().info("Verifying the actual joint positions ...")
        deadline = time.monotonic() + args.verify_timeout
        sequence_at_start = self._joint_state_sequence
        errors = None

        while time.monotonic() < deadline:
            # Require state feedback after MoveIt reports execution completion.
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self._joint_positions is None
                or self._joint_state_sequence <= sequence_at_start
            ):
                continue

            errors = {
                name: abs(self._joint_positions[name] - target)
                for name, target in zip(UR_JOINT_NAMES, target_positions)
            }
            maximum_error = max(errors.values())
            if maximum_error <= args.verify_joint_tolerance:
                self.get_logger().info(
                    "Goal verification succeeded: maximum_joint_error="
                    f"{maximum_error:.6f} rad, tolerance="
                    f"{args.verify_joint_tolerance:.6f} rad"
                )
                return True

        if self._joint_positions is None or errors is None:
            self.get_logger().error(
                "Goal verification failed: no complete /joint_states message received."
            )
            return False

        self.get_logger().error(
            "Goal verification failed: the actual joints did not reach the target "
            f"within {args.verify_timeout:.1f} s."
        )
        for name, target in zip(UR_JOINT_NAMES, target_positions):
            actual = self._joint_positions[name]
            error = errors[name]
            marker = "FAILED" if error > args.verify_joint_tolerance else "OK"
            self.get_logger().error(
                f"  {name}: target={target:.6f}, actual={actual:.6f}, "
                f"error={error:.6f} rad [{marker}]"
            )
        return False

    @staticmethod
    def _positions_in_radians(args: argparse.Namespace) -> list[float]:
        positions = list(args.joints)
        if args.degrees:
            positions = [math.radians(position) for position in positions]
        return positions

    @staticmethod
    def _make_goal(
        args: argparse.Namespace, positions: list[float]
    ) -> MoveGroup.Goal:
        constraints = Constraints()
        for name, position in zip(UR_JOINT_NAMES, positions):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = args.joint_tolerance
            constraint.tolerance_below = args.joint_tolerance
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)

        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = args.group
        request.num_planning_attempts = args.planning_attempts
        request.allowed_planning_time = args.planning_time
        request.max_velocity_scaling_factor = args.velocity_scaling
        request.max_acceleration_scaling_factor = args.acceleration_scaling
        request.start_state.is_diff = True
        request.goal_constraints = [constraints]

        # Execution is deliberately separate so the inspected plan is the one run.
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        return goal


def finite_number(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError("must be a finite number")
    return parsed


def unit_interval(value: str) -> float:
    parsed = finite_number(value)
    if not 0.0 < parsed <= 1.0:
        raise argparse.ArgumentTypeError("must be in the interval (0, 1]")
    return parsed


def positive(value: str) -> float:
    parsed = finite_number(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plan (default) or execute one UR3 joint goal through MoveIt 2."
    )
    parser.add_argument(
        "joints",
        metavar="J",
        nargs=6,
        type=finite_number,
        help="J1 through J6, in radians by default",
    )
    parser.add_argument(
        "--degrees",
        action="store_true",
        help="Interpret J1 through J6 as degrees instead of radians.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Execute on the configured controller. Without this flag, plan only.",
    )
    parser.add_argument("--velocity-scaling", type=unit_interval, default=0.1)
    parser.add_argument("--acceleration-scaling", type=unit_interval, default=0.1)
    parser.add_argument("--planning-time", type=positive, default=5.0)
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--joint-tolerance", type=positive, default=0.001)
    parser.add_argument("--verify-joint-tolerance", type=positive, default=0.01)
    parser.add_argument("--verify-timeout", type=positive, default=3.0)
    parser.add_argument(
        "--max-joint-travel",
        type=positive,
        default=0.5,
        help="Block execution if any joint travels farther than this many radians.",
    )
    parser.add_argument("--server-timeout", type=positive, default=10.0)
    parser.add_argument("--group", default="ur_manipulator")
    args = parser.parse_args(argv)
    if args.planning_attempts < 1:
        parser.error("--planning-attempts must be at least 1")
    return args


def main(args=None) -> None:
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    cli_args = parse_args(rclpy.utilities.remove_ros_args(raw_args)[1:])
    rclpy.init(args=args)
    node = MoveToJointOnce()
    try:
        success = node.run(cli_args)
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted by user.")
        success = False
    except Exception as exc:
        node.get_logger().error(f"Unexpected error: {type(exc).__name__}: {exc}")
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

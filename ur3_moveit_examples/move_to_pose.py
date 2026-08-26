#!/usr/bin/env python3
"""Send one pose goal to MoveIt's MoveGroup action.

This node never talks to FollowJointTrajectory directly.  With plan_only=False,
move_group selects its configured controller (the real UR config defaults to
scaled_joint_trajectory_controller) and monitors execution.
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from ur3_moveit_examples.trajectory_safety import (
    joint_excursions,
    trajectory_duration_seconds,
)


ERROR_NAMES = {
    value: name
    for name, value in vars(MoveItErrorCodes).items()
    if name.isupper() and isinstance(value, int)
}

UR_JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


class MoveToPoseOnce(Node):
    def __init__(self) -> None:
        super().__init__("ur3_move_to_pose_once")
        self._client = ActionClient(self, MoveGroup, "/move_action")
        self._execute_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self._joint_positions: dict[str, float] | None = None
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _joint_state_callback(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in UR_JOINT_NAMES):
            self._joint_positions = {
                name: positions[name] for name in UR_JOINT_NAMES
            }

    def _wait_for_joint_state(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while self._joint_positions is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._joint_positions is None:
            self.get_logger().error(
                "No complete UR3 joint state received on /joint_states."
            )
            return False
        return True

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
        if not self._wait_for_joint_state(args.server_timeout):
            return False
        if not self._ik_client.wait_for_service(timeout_sec=args.server_timeout):
            self.get_logger().error("MoveIt /compute_ik service was not found.")
            return False

        target = self._target_pose(args)
        ik_positions = self._compute_nearby_ik(target, args)
        if ik_positions is None:
            return False

        goal = self._make_goal(args, ik_positions)
        mode = "PLAN FOR EXECUTION" if args.execute else "PLAN ONLY"
        self.get_logger().warn(
            f"{mode}: tool0 target in base_link = "
            f"position({args.x:.4f}, {args.y:.4f}, {args.z:.4f}), "
            f"quaternion({args.qx:.5f}, {args.qy:.5f}, {args.qz:.5f}, {args.qw:.5f})"
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
            return self._verify_pose_goal(target, args)
        return True

    def _compute_nearby_ik(
        self, target: Pose, args: argparse.Namespace
    ) -> list[float] | None:
        request = GetPositionIK.Request()
        ik_request = request.ik_request
        ik_request.group_name = args.group
        ik_request.ik_link_name = args.ee_link
        ik_request.avoid_collisions = True
        ik_request.pose_stamped.header.frame_id = args.frame_id
        ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ik_request.pose_stamped.pose = target
        ik_request.robot_state.joint_state.name = list(UR_JOINT_NAMES)
        ik_request.robot_state.joint_state.position = [
            self._joint_positions[name] for name in UR_JOINT_NAMES
        ]
        ik_request.robot_state.is_diff = False
        ik_request.timeout.sec = int(args.ik_timeout)
        ik_request.timeout.nanosec = int(
            (args.ik_timeout - int(args.ik_timeout)) * 1.0e9
        )

        self.get_logger().info("Computing collision-aware IK from the current state ...")
        future = self._ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error("No response received from /compute_ik.")
            return None

        code = response.error_code.val
        code_name = ERROR_NAMES.get(code, f"UNKNOWN_ERROR_{code}")
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"IK failed: {code_name} ({code})")
            return None

        solution_by_name = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
        )
        if not all(name in solution_by_name for name in UR_JOINT_NAMES):
            self.get_logger().error("IK response does not contain all UR3 joints.")
            return None

        positions = [solution_by_name[name] for name in UR_JOINT_NAMES]
        differences = {
            name: abs(solution - self._joint_positions[name])
            for name, solution in zip(UR_JOINT_NAMES, positions)
        }
        maximum_name = max(differences, key=differences.get)
        maximum = differences[maximum_name]
        self.get_logger().info(
            f"IK succeeded: maximum_joint_difference={maximum:.6f} rad "
            f"({maximum_name})"
        )
        for name, solution in zip(UR_JOINT_NAMES, positions):
            self.get_logger().info(
                f"  {name}: current={self._joint_positions[name]:.6f}, "
                f"IK={solution:.6f}, difference={differences[name]:.6f} rad"
            )

        if maximum > args.max_joint_travel:
            self.get_logger().error(
                "Planning blocked: IK solution is too far from the current state. "
                f"maximum={maximum:.6f} rad, limit="
                f"{args.max_joint_travel:.6f} rad, joint={maximum_name}"
            )
            return None
        return positions

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

    def _verify_pose_goal(self, target: Pose, args: argparse.Namespace) -> bool:
        self.get_logger().info(
            f"Verifying the actual TCP pose {args.frame_id} -> {args.ee_link} ..."
        )
        deadline = time.monotonic() + args.verify_timeout
        last_tf_error = "transform has not been received"
        position_error = None
        orientation_error = None

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                transform = self._tf_buffer.lookup_transform(
                    args.frame_id,
                    args.ee_link,
                    rclpy.time.Time(),
                )
            except TransformException as exc:
                last_tf_error = str(exc)
                continue

            actual_position = transform.transform.translation
            actual_orientation = transform.transform.rotation
            position_error = math.sqrt(
                (actual_position.x - target.position.x) ** 2
                + (actual_position.y - target.position.y) ** 2
                + (actual_position.z - target.position.z) ** 2
            )
            dot = abs(
                actual_orientation.x * target.orientation.x
                + actual_orientation.y * target.orientation.y
                + actual_orientation.z * target.orientation.z
                + actual_orientation.w * target.orientation.w
            )
            orientation_error = 2.0 * math.acos(max(0.0, min(1.0, dot)))

            if (
                position_error <= args.verify_position_tolerance
                and orientation_error <= args.verify_orientation_tolerance
            ):
                self.get_logger().info(
                    "Goal verification succeeded: position_error="
                    f"{position_error:.6f} m, orientation_error="
                    f"{orientation_error:.6f} rad"
                )
                return True

        if position_error is None or orientation_error is None:
            self.get_logger().error(
                f"Goal verification failed: could not look up TF "
                f"{args.frame_id} -> {args.ee_link}: {last_tf_error}"
            )
            return False

        self.get_logger().error(
            "Goal verification failed: the actual TCP did not reach the target "
            f"within {args.verify_timeout:.1f} s. position_error="
            f"{position_error:.6f} m (tolerance "
            f"{args.verify_position_tolerance:.6f} m), orientation_error="
            f"{orientation_error:.6f} rad (tolerance "
            f"{args.verify_orientation_tolerance:.6f} rad)"
        )
        return False

    @staticmethod
    def _target_pose(args: argparse.Namespace) -> Pose:
        target = Pose()
        target.position.x = args.x
        target.position.y = args.y
        target.position.z = args.z

        norm = math.sqrt(args.qx**2 + args.qy**2 + args.qz**2 + args.qw**2)
        target.orientation.x = args.qx / norm
        target.orientation.y = args.qy / norm
        target.orientation.z = args.qz / norm
        target.orientation.w = args.qw / norm
        return target

    @staticmethod
    def _make_goal(
        args: argparse.Namespace,
        ik_positions: list[float],
    ) -> MoveGroup.Goal:
        constraints = Constraints()
        for name, position in zip(UR_JOINT_NAMES, ik_positions):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = position
            joint.tolerance_above = args.ik_joint_tolerance
            joint.tolerance_below = args.ik_joint_tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)

        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = args.group
        request.num_planning_attempts = args.planning_attempts
        request.allowed_planning_time = args.planning_time
        request.max_velocity_scaling_factor = args.velocity_scaling
        request.max_acceleration_scaling_factor = args.acceleration_scaling
        # An empty diff tells MoveIt to use the current monitored robot state.
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
        description="Plan (default) or execute one UR3 TCP pose through MoveIt 2."
    )
    parser.add_argument("x", type=finite_number)
    parser.add_argument("y", type=finite_number)
    parser.add_argument("z", type=finite_number)
    parser.add_argument("qx", type=finite_number)
    parser.add_argument("qy", type=finite_number)
    parser.add_argument("qz", type=finite_number)
    parser.add_argument("qw", type=finite_number)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Execute on the configured controller. Without this flag, plan only.",
    )
    parser.add_argument("--velocity-scaling", type=unit_interval, default=0.1)
    parser.add_argument("--acceleration-scaling", type=unit_interval, default=0.1)
    parser.add_argument("--planning-time", type=positive, default=5.0)
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--ik-timeout", type=positive, default=1.0)
    parser.add_argument("--ik-joint-tolerance", type=positive, default=0.001)
    parser.add_argument("--verify-position-tolerance", type=positive, default=0.005)
    parser.add_argument("--verify-orientation-tolerance", type=positive, default=0.02)
    parser.add_argument("--verify-timeout", type=positive, default=3.0)
    parser.add_argument(
        "--max-joint-travel",
        type=positive,
        default=0.5,
        help=(
            "Reject distant pose IK solutions and block execution if any joint "
            "travels farther than this many radians."
        ),
    )
    parser.add_argument("--server-timeout", type=positive, default=10.0)
    parser.add_argument("--group", default="ur_manipulator")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--ee-link", default="tool0")
    args = parser.parse_args(argv)
    norm = math.sqrt(args.qx**2 + args.qy**2 + args.qz**2 + args.qw**2)
    if norm < 1.0e-9:
        parser.error("quaternion norm must be non-zero")
    if args.planning_attempts < 1:
        parser.error("--planning-attempts must be at least 1")
    return args


def main(args=None) -> None:
    # Remove ROS arguments so argparse only sees this program's arguments.
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    cli_args = parse_args(rclpy.utilities.remove_ros_args(raw_args)[1:])
    rclpy.init(args=args)
    node = MoveToPoseOnce()
    try:
        success = node.run(cli_args)
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted by user.")
        success = False
    except Exception as exc:  # Ensure an actionable error is printed at the CLI.
        node.get_logger().error(f"Unexpected error: {type(exc).__name__}: {exc}")
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

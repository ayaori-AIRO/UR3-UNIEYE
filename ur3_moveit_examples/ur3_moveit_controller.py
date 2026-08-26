"""Reusable MoveIt 2 client for a real UR3 on ROS 2 Humble."""

import math
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

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


class UR3MoveItController(Node):
    """Plan, inspect, execute, and verify UR3 motions through MoveIt 2."""

    def __init__(
        self,
        *,
        node_name: str = "ur3_moveit_controller",
        group: str = "ur_manipulator",
        base_frame: str = "base_link",
        ee_link: str = "tool0",
        velocity_scaling: float = 0.1,
        acceleration_scaling: float = 0.1,
        planning_time: float = 5.0,
        planning_attempts: int = 5,
        max_joint_travel: float = 0.5,
        joint_goal_tolerance: float = 0.001,
        ik_timeout: float = 1.0,
        ik_joint_tolerance: float = 0.001,
        verify_joint_tolerance: float = 0.01,
        verify_position_tolerance: float = 0.005,
        verify_orientation_tolerance: float = 0.02,
        verify_timeout: float = 3.0,
        server_timeout: float = 10.0,
    ) -> None:
        super().__init__(node_name)
        self.group = group
        self.base_frame = base_frame
        self.ee_link = ee_link
        self.velocity_scaling = velocity_scaling
        self.acceleration_scaling = acceleration_scaling
        self.planning_time = planning_time
        self.planning_attempts = planning_attempts
        self.max_joint_travel = max_joint_travel
        self.joint_goal_tolerance = joint_goal_tolerance
        self.ik_timeout = ik_timeout
        self.ik_joint_tolerance = ik_joint_tolerance
        self.verify_joint_tolerance = verify_joint_tolerance
        self.verify_position_tolerance = verify_position_tolerance
        self.verify_orientation_tolerance = verify_orientation_tolerance
        self.verify_timeout = verify_timeout
        self.server_timeout = server_timeout
        self._validate_configuration()

        self._move_client = ActionClient(self, MoveGroup, "/move_action")
        self._execute_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self._cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        self._joint_positions: dict[str, float] | None = None
        self._joint_state_sequence = 0
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _validate_configuration(self) -> None:
        for name, value in (
            ("velocity_scaling", self.velocity_scaling),
            ("acceleration_scaling", self.acceleration_scaling),
        ):
            if not 0.0 < value <= 1.0:
                raise ValueError(f"{name} must be in (0, 1]")
        for name, value in (
            ("planning_time", self.planning_time),
            ("max_joint_travel", self.max_joint_travel),
            ("joint_goal_tolerance", self.joint_goal_tolerance),
            ("ik_timeout", self.ik_timeout),
            ("ik_joint_tolerance", self.ik_joint_tolerance),
            ("verify_joint_tolerance", self.verify_joint_tolerance),
            ("verify_position_tolerance", self.verify_position_tolerance),
            ("verify_orientation_tolerance", self.verify_orientation_tolerance),
            ("verify_timeout", self.verify_timeout),
            ("server_timeout", self.server_timeout),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be a positive finite number")
        if self.planning_attempts < 1:
            raise ValueError("planning_attempts must be at least 1")

    def _joint_state_callback(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in UR_JOINT_NAMES):
            self._joint_positions = {
                name: positions[name] for name in UR_JOINT_NAMES
            }
            self._joint_state_sequence += 1

    def get_current_joint_positions(
        self, timeout: float | None = None
    ) -> dict[str, float] | None:
        deadline = time.monotonic() + (timeout or self.server_timeout)
        while self._joint_positions is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._joint_positions is None:
            self.get_logger().error(
                "No complete UR3 joint state received on /joint_states."
            )
            return None
        return dict(self._joint_positions)

    def get_current_pose(self, timeout: float | None = None) -> PoseStamped | None:
        deadline = time.monotonic() + (timeout or self.server_timeout)
        last_error = "transform has not been received"
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                transform = self._tf_buffer.lookup_transform(
                    self.base_frame, self.ee_link, rclpy.time.Time()
                )
            except TransformException as exc:
                last_error = str(exc)
                continue
            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose
        self.get_logger().error(
            f"Could not look up TF {self.base_frame} -> {self.ee_link}: {last_error}"
        )
        return None

    def move_to_joint(
        self, joint_positions: list[float], *, execute: bool = False
    ) -> bool:
        if len(joint_positions) != len(UR_JOINT_NAMES):
            raise ValueError("joint_positions must contain exactly 6 values")
        if not all(math.isfinite(value) for value in joint_positions):
            raise ValueError("joint_positions must contain finite values")
        if not self._wait_for_moveit(execute=execute, needs_ik=False):
            return False

        mode = "PLAN FOR EXECUTION" if execute else "PLAN ONLY"
        self.get_logger().warn(f"{mode}: joint target [rad]")
        for name, position in zip(UR_JOINT_NAMES, joint_positions):
            self.get_logger().warn(
                f"  {name}: {position:.6f} rad ({math.degrees(position):.3f} deg)"
            )

        result = self._plan_joint_goal(joint_positions, self.joint_goal_tolerance)
        if result is None or not self._inspect_trajectory(result, execute):
            return False
        if not execute:
            return True
        if not self._execute_trajectory(result.planned_trajectory):
            return False
        return self._verify_joint_goal(joint_positions)

    def move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        *,
        execute: bool = False,
    ) -> bool:
        target = self._make_pose(x, y, z, qx, qy, qz, qw)
        if not self._wait_for_moveit(execute=execute, needs_ik=True):
            return False
        if self.get_current_joint_positions() is None:
            return False
        ik_positions = self._compute_nearby_ik(target)
        if ik_positions is None:
            return False

        mode = "PLAN FOR EXECUTION" if execute else "PLAN ONLY"
        self.get_logger().warn(
            f"{mode}: {self.ee_link} target in {self.base_frame} = "
            f"position({x:.4f}, {y:.4f}, {z:.4f}), "
            f"quaternion({target.orientation.x:.5f}, "
            f"{target.orientation.y:.5f}, {target.orientation.z:.5f}, "
            f"{target.orientation.w:.5f})"
        )
        result = self._plan_joint_goal(ik_positions, self.ik_joint_tolerance)
        if result is None or not self._inspect_trajectory(result, execute):
            return False
        if not execute:
            return True
        if not self._execute_trajectory(result.planned_trajectory):
            return False
        return self._verify_pose_goal(target)

    def move_relative_tool(
        self, dx: float, dy: float, dz: float, *, execute: bool = False
    ) -> bool:
        """Move by a translation expressed in the current tool frame.

        The current TCP orientation is preserved. Planning and execution use the
        same collision-aware IK and trajectory safety checks as move_to_pose().
        """
        offsets = (dx, dy, dz)
        if not all(math.isfinite(value) for value in offsets):
            raise ValueError("tool-relative offsets must be finite")

        current = self.get_current_pose()
        if current is None:
            return False
        orientation = current.pose.orientation
        base_dx, base_dy, base_dz = self._rotate_vector_by_quaternion(
            dx,
            dy,
            dz,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        target = current.pose
        target.position.x += base_dx
        target.position.y += base_dy
        target.position.z += base_dz

        self.get_logger().info(
            f"Tool-relative translation: dx={dx:.6f}, dy={dy:.6f}, "
            f"dz={dz:.6f} m -> {self.base_frame} translation: "
            f"dx={base_dx:.6f}, dy={base_dy:.6f}, dz={base_dz:.6f} m"
        )
        return self.move_to_pose(
            target.position.x,
            target.position.y,
            target.position.z,
            target.orientation.x,
            target.orientation.y,
            target.orientation.z,
            target.orientation.w,
            execute=execute,
        )

    def move_cartesian_relative_tool(
        self,
        dx: float,
        dy: float,
        dz: float,
        *,
        execute: bool = False,
        max_step: float = 0.001,
        jump_threshold: float = 2.0,
        minimum_fraction: float = 0.999,
    ) -> bool:
        """Move the TCP along a collision-checked straight Cartesian path."""
        offsets = (dx, dy, dz)
        if not all(math.isfinite(value) for value in offsets):
            raise ValueError("tool-relative offsets must be finite")
        current = self.get_current_pose()
        if current is None:
            return False

        orientation = current.pose.orientation
        base_dx, base_dy, base_dz = self._rotate_vector_by_quaternion(
            dx,
            dy,
            dz,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        target = current.pose
        target.position.x += base_dx
        target.position.y += base_dy
        target.position.z += base_dz
        self.get_logger().info(
            f"Cartesian tool translation: dx={dx:.6f}, dy={dy:.6f}, "
            f"dz={dz:.6f} m -> {self.base_frame} translation: "
            f"dx={base_dx:.6f}, dy={base_dy:.6f}, dz={base_dz:.6f} m"
        )
        return self.move_cartesian_to_pose(
            target.position.x,
            target.position.y,
            target.position.z,
            target.orientation.x,
            target.orientation.y,
            target.orientation.z,
            target.orientation.w,
            execute=execute,
            max_step=max_step,
            jump_threshold=jump_threshold,
            minimum_fraction=minimum_fraction,
        )

    def move_cartesian_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        *,
        execute: bool = False,
        max_step: float = 0.001,
        jump_threshold: float = 2.0,
        minimum_fraction: float = 0.999,
    ) -> bool:
        """Move in a straight Cartesian path to an absolute TCP pose."""
        target = self._make_pose(x, y, z, qx, qy, qz, qw)
        return self._move_cartesian_to_pose(
            target,
            execute=execute,
            max_step=max_step,
            jump_threshold=jump_threshold,
            minimum_fraction=minimum_fraction,
        )

    def _move_cartesian_to_pose(
        self,
        target: Pose,
        *,
        execute: bool,
        max_step: float,
        jump_threshold: float,
        minimum_fraction: float,
    ) -> bool:
        if not math.isfinite(max_step) or max_step <= 0.0:
            raise ValueError("max_step must be a positive finite number")
        if not math.isfinite(jump_threshold) or jump_threshold < 0.0:
            raise ValueError("jump_threshold must be a non-negative finite number")
        if not 0.0 < minimum_fraction <= 1.0:
            raise ValueError("minimum_fraction must be in (0, 1]")
        if not self._wait_for_cartesian(execute=execute):
            return False
        joints = self.get_current_joint_positions()
        if joints is None:
            return False

        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state.joint_state.header = request.header
        request.start_state.joint_state.name = list(UR_JOINT_NAMES)
        request.start_state.joint_state.position = [
            joints[name] for name in UR_JOINT_NAMES
        ]
        request.start_state.is_diff = False
        request.group_name = self.group
        request.link_name = self.ee_link
        request.waypoints = [target]
        request.max_step = max_step
        request.jump_threshold = jump_threshold
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = self.max_joint_travel
        request.avoid_collisions = True

        mode = "PLAN FOR EXECUTION" if execute else "PLAN ONLY"
        self.get_logger().warn(
            f"{mode}: straight Cartesian path; max_step={max_step:.6f} m, "
            f"jump_threshold={jump_threshold:.3f}"
        )
        future = self._cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error("No response received from /compute_cartesian_path.")
            return False
        code = response.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Cartesian planning failed: {self._error_name(code)} ({code})"
            )
            return False
        self.get_logger().info(
            f"Cartesian path computed: fraction={response.fraction:.6f}, "
            f"trajectory_points={len(response.solution.joint_trajectory.points)}"
        )
        if response.fraction < minimum_fraction:
            self.get_logger().error(
                "Cartesian path rejected: incomplete path. "
                f"fraction={response.fraction:.6f}, required={minimum_fraction:.6f}"
            )
            return False

        self._slow_cartesian_trajectory(response.solution)
        if not self._inspect_robot_trajectory(
            response.start_state, response.solution, execute
        ):
            return False
        if not execute:
            return True
        if not self._execute_trajectory(response.solution):
            return False
        return self._verify_pose_goal(target)

    def _wait_for_cartesian(self, *, execute: bool) -> bool:
        self.get_logger().info("Waiting for MoveIt /compute_cartesian_path ...")
        if not self._cartesian_client.wait_for_service(
            timeout_sec=self.server_timeout
        ):
            self.get_logger().error(
                "MoveIt /compute_cartesian_path service was not found."
            )
            return False
        if execute and not self._execute_client.wait_for_server(
            timeout_sec=self.server_timeout
        ):
            self.get_logger().error("MoveIt /execute_trajectory server was not found.")
            return False
        return True

    def _slow_cartesian_trajectory(self, trajectory) -> None:
        # Humble's Cartesian service time-parameterizes at scaling 1.0. Stretch
        # time so both configured velocity and acceleration limits are respected.
        speed_scale = min(
            self.velocity_scaling, math.sqrt(self.acceleration_scaling)
        )
        for point in trajectory.joint_trajectory.points:
            duration = point.time_from_start
            nanoseconds = duration.sec * 1_000_000_000 + duration.nanosec
            scaled_nanoseconds = int(round(nanoseconds / speed_scale))
            duration.sec = scaled_nanoseconds // 1_000_000_000
            duration.nanosec = scaled_nanoseconds % 1_000_000_000
            point.velocities = [value * speed_scale for value in point.velocities]
            point.accelerations = [
                value * speed_scale**2 for value in point.accelerations
            ]
        self.get_logger().info(
            f"Cartesian trajectory timing scaled by {speed_scale:.6f} "
            f"(velocity_limit={self.velocity_scaling:.6f}, "
            f"acceleration_limit={self.acceleration_scaling:.6f})"
        )

    @staticmethod
    def _rotate_vector_by_quaternion(
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> tuple[float, float, float]:
        norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        if norm < 1.0e-9:
            raise ValueError("current TCP quaternion norm must be non-zero")
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        # Unit-quaternion rotation matrix, mapping tool-frame vectors to base.
        return (
            (1.0 - 2.0 * (qy * qy + qz * qz)) * x
            + 2.0 * (qx * qy - qz * qw) * y
            + 2.0 * (qx * qz + qy * qw) * z,
            2.0 * (qx * qy + qz * qw) * x
            + (1.0 - 2.0 * (qx * qx + qz * qz)) * y
            + 2.0 * (qy * qz - qx * qw) * z,
            2.0 * (qx * qz - qy * qw) * x
            + 2.0 * (qy * qz + qx * qw) * y
            + (1.0 - 2.0 * (qx * qx + qy * qy)) * z,
        )

    def _wait_for_moveit(self, *, execute: bool, needs_ik: bool) -> bool:
        self.get_logger().info("Waiting for MoveIt /move_action ...")
        if not self._move_client.wait_for_server(timeout_sec=self.server_timeout):
            self.get_logger().error("MoveIt /move_action server was not found.")
            return False
        if execute and not self._execute_client.wait_for_server(
            timeout_sec=self.server_timeout
        ):
            self.get_logger().error("MoveIt /execute_trajectory server was not found.")
            return False
        if needs_ik and not self._ik_client.wait_for_service(
            timeout_sec=self.server_timeout
        ):
            self.get_logger().error("MoveIt /compute_ik service was not found.")
            return False
        return True

    @staticmethod
    def _make_pose(
        x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float
    ) -> Pose:
        values = (x, y, z, qx, qy, qz, qw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("pose values must be finite")
        norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        if norm < 1.0e-9:
            raise ValueError("quaternion norm must be non-zero")
        target = Pose()
        target.position.x, target.position.y, target.position.z = x, y, z
        target.orientation.x = qx / norm
        target.orientation.y = qy / norm
        target.orientation.z = qz / norm
        target.orientation.w = qw / norm
        return target

    def _compute_nearby_ik(self, target: Pose) -> list[float] | None:
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = self.group
        ik.ik_link_name = self.ee_link
        ik.avoid_collisions = True
        ik.pose_stamped.header.frame_id = self.base_frame
        ik.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ik.pose_stamped.pose = target
        ik.robot_state.joint_state.name = list(UR_JOINT_NAMES)
        ik.robot_state.joint_state.position = [
            self._joint_positions[name] for name in UR_JOINT_NAMES
        ]
        ik.robot_state.is_diff = False
        ik.timeout.sec = int(self.ik_timeout)
        ik.timeout.nanosec = int((self.ik_timeout - int(self.ik_timeout)) * 1.0e9)

        self.get_logger().info("Computing collision-aware IK from the current state ...")
        future = self._ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error("No response received from /compute_ik.")
            return None
        code = response.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"IK failed: {self._error_name(code)} ({code})")
            return None

        solution = dict(
            zip(response.solution.joint_state.name, response.solution.joint_state.position)
        )
        if not all(name in solution for name in UR_JOINT_NAMES):
            self.get_logger().error("IK response does not contain all UR3 joints.")
            return None
        positions = [solution[name] for name in UR_JOINT_NAMES]
        differences = {
            name: abs(value - self._joint_positions[name])
            for name, value in zip(UR_JOINT_NAMES, positions)
        }
        maximum_name = max(differences, key=differences.get)
        maximum = differences[maximum_name]
        self.get_logger().info(
            f"IK succeeded: maximum_joint_difference={maximum:.6f} rad "
            f"({maximum_name})"
        )
        for name, value in zip(UR_JOINT_NAMES, positions):
            self.get_logger().info(
                f"  {name}: current={self._joint_positions[name]:.6f}, "
                f"IK={value:.6f}, difference={differences[name]:.6f} rad"
            )
        if maximum > self.max_joint_travel:
            self.get_logger().error(
                "Planning blocked: IK solution is too far from the current state. "
                f"maximum={maximum:.6f} rad, limit={self.max_joint_travel:.6f} "
                f"rad, joint={maximum_name}"
            )
            return None
        return positions

    def _plan_joint_goal(self, positions: list[float], tolerance: float):
        constraints = Constraints()
        for name, position in zip(UR_JOINT_NAMES, positions):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = position
            joint.tolerance_above = tolerance
            joint.tolerance_below = tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)

        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.group
        request.num_planning_attempts = self.planning_attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling
        request.start_state.is_diff = True
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True

        send_future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("MoveIt rejected the planning goal.")
            return None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error("No planning result received from MoveIt.")
            return None
        result = wrapped.result
        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"MoveIt planning failed: {self._error_name(code)} ({code}); "
                f"planning_time={result.planning_time:.3f} s"
            )
            return None
        count = len(result.planned_trajectory.joint_trajectory.points)
        self.get_logger().info(
            f"MoveIt planning succeeded: {self._error_name(code)}; "
            f"planning_time={result.planning_time:.3f} s; trajectory_points={count}"
        )
        return result

    def _inspect_trajectory(self, result, execute: bool) -> bool:
        return self._inspect_robot_trajectory(
            result.trajectory_start, result.planned_trajectory, execute
        )

    def _inspect_robot_trajectory(
        self, trajectory_start, trajectory, execute: bool
    ) -> bool:
        excursions = joint_excursions(trajectory_start, trajectory)
        if not excursions:
            self.get_logger().error("Planned trajectory contains no joint points.")
            return False
        maximum_name = max(excursions, key=excursions.get)
        maximum = excursions[maximum_name]
        self.get_logger().info(
            f"Trajectory inspection: duration="
            f"{trajectory_duration_seconds(trajectory):.3f} s, "
            f"maximum_joint_travel={maximum:.6f} rad ({maximum_name})"
        )
        for name, travel in excursions.items():
            self.get_logger().info(f"  {name}: maximum_travel={travel:.6f} rad")
        if execute and maximum > self.max_joint_travel:
            self.get_logger().error(
                "Execution blocked: planned joint travel exceeds the configured "
                f"limit. maximum={maximum:.6f} rad, limit="
                f"{self.max_joint_travel:.6f} rad, joint={maximum_name}"
            )
            return False
        return True

    def _execute_trajectory(self, trajectory) -> bool:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        self.get_logger().warn("Executing the exact inspected trajectory ...")
        send_future = self._execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("MoveIt rejected the trajectory execution goal.")
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error("No trajectory execution result received.")
            return False
        code = wrapped.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Trajectory execution failed: {self._error_name(code)} ({code})"
            )
            return False
        self.get_logger().info(
            f"Trajectory execution succeeded: {self._error_name(code)}"
        )
        return True

    def _verify_joint_goal(self, targets: list[float]) -> bool:
        self.get_logger().info("Verifying the actual joint positions ...")
        deadline = time.monotonic() + self.verify_timeout
        sequence = self._joint_state_sequence
        errors = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._joint_state_sequence <= sequence or self._joint_positions is None:
                continue
            errors = {
                name: abs(self._joint_positions[name] - target)
                for name, target in zip(UR_JOINT_NAMES, targets)
            }
            maximum = max(errors.values())
            if maximum <= self.verify_joint_tolerance:
                self.get_logger().info(
                    f"Goal verification succeeded: maximum_joint_error="
                    f"{maximum:.6f} rad, tolerance="
                    f"{self.verify_joint_tolerance:.6f} rad"
                )
                return True
        if errors is None:
            self.get_logger().error("Goal verification failed: no new joint state.")
            return False
        self.get_logger().error("Goal verification failed: joint target not reached.")
        for name, target in zip(UR_JOINT_NAMES, targets):
            actual = self._joint_positions[name]
            self.get_logger().error(
                f"  {name}: target={target:.6f}, actual={actual:.6f}, "
                f"error={errors[name]:.6f} rad"
            )
        return False

    def _verify_pose_goal(self, target: Pose) -> bool:
        self.get_logger().info(
            f"Verifying the actual TCP pose {self.base_frame} -> {self.ee_link} ..."
        )
        deadline = time.monotonic() + self.verify_timeout
        position_error = orientation_error = None
        while time.monotonic() < deadline:
            current = self.get_current_pose(timeout=0.2)
            if current is None:
                continue
            position_error = math.sqrt(
                (current.pose.position.x - target.position.x) ** 2
                + (current.pose.position.y - target.position.y) ** 2
                + (current.pose.position.z - target.position.z) ** 2
            )
            actual = current.pose.orientation
            dot = abs(
                actual.x * target.orientation.x
                + actual.y * target.orientation.y
                + actual.z * target.orientation.z
                + actual.w * target.orientation.w
            )
            orientation_error = 2.0 * math.acos(max(0.0, min(1.0, dot)))
            if (
                position_error <= self.verify_position_tolerance
                and orientation_error <= self.verify_orientation_tolerance
            ):
                self.get_logger().info(
                    f"Goal verification succeeded: position_error="
                    f"{position_error:.6f} m, orientation_error="
                    f"{orientation_error:.6f} rad"
                )
                return True
        self.get_logger().error(
            "Goal verification failed: TCP target not reached. position_error="
            f"{position_error}, orientation_error={orientation_error}"
        )
        return False

    @staticmethod
    def _error_name(code: int) -> str:
        return ERROR_NAMES.get(code, f"UNKNOWN_ERROR_{code}")

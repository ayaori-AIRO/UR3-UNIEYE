#!/usr/bin/env python3
"""Send one pose goal to MoveIt's MoveGroup action.

This node never talks to FollowJointTrajectory directly.  With plan_only=False,
move_group selects its configured controller (the real UR config defaults to
scaled_joint_trajectory_controller) and monitors execution.
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, MoveItErrorCodes, OrientationConstraint
from moveit_msgs.msg import PositionConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


ERROR_NAMES = {
    value: name
    for name, value in vars(MoveItErrorCodes).items()
    if name.isupper() and isinstance(value, int)
}


class MoveToPoseOnce(Node):
    def __init__(self) -> None:
        super().__init__("ur3_move_to_pose_once")
        self._client = ActionClient(self, MoveGroup, "/move_action")

    def run(self, args: argparse.Namespace) -> bool:
        self.get_logger().info("Waiting for MoveIt /move_action ...")
        if not self._client.wait_for_server(timeout_sec=args.server_timeout):
            self.get_logger().error(
                "MoveIt action server was not found. Is ur_moveit.launch.py running?"
            )
            return False

        goal = self._make_goal(args)
        mode = "PLAN + EXECUTE" if args.execute else "PLAN ONLY"
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
            f"MoveIt succeeded: {code_name}; planning_time="
            f"{result.planning_time:.3f} s; trajectory_points={point_count}; mode={mode}"
        )
        return True

    @staticmethod
    def _make_goal(args: argparse.Namespace) -> MoveGroup.Goal:
        target = Pose()
        target.position.x = args.x
        target.position.y = args.y
        target.position.z = args.z

        norm = math.sqrt(args.qx**2 + args.qy**2 + args.qz**2 + args.qw**2)
        target.orientation.x = args.qx / norm
        target.orientation.y = args.qy / norm
        target.orientation.z = args.qz / norm
        target.orientation.w = args.qw / norm

        position_region = SolidPrimitive()
        position_region.type = SolidPrimitive.SPHERE
        position_region.dimensions = [args.position_tolerance]

        position = PositionConstraint()
        position.header.frame_id = args.frame_id
        position.link_name = args.ee_link
        position.constraint_region.primitives = [position_region]
        position.constraint_region.primitive_poses = [target]
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = args.frame_id
        orientation.link_name = args.ee_link
        orientation.orientation = target.orientation
        orientation.absolute_x_axis_tolerance = args.orientation_tolerance
        orientation.absolute_y_axis_tolerance = args.orientation_tolerance
        orientation.absolute_z_axis_tolerance = args.orientation_tolerance
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]

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

        goal.planning_options.plan_only = not args.execute
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        return goal


def unit_interval(value: str) -> float:
    parsed = float(value)
    if not 0.0 < parsed <= 1.0:
        raise argparse.ArgumentTypeError("must be in the interval (0, 1]")
    return parsed


def positive(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plan (default) or execute one UR3 TCP pose through MoveIt 2."
    )
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("z", type=float)
    parser.add_argument("qx", type=float)
    parser.add_argument("qy", type=float)
    parser.add_argument("qz", type=float)
    parser.add_argument("qw", type=float)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Execute on the configured controller. Without this flag, plan only.",
    )
    parser.add_argument("--velocity-scaling", type=unit_interval, default=0.1)
    parser.add_argument("--acceleration-scaling", type=unit_interval, default=0.1)
    parser.add_argument("--planning-time", type=positive, default=5.0)
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--position-tolerance", type=positive, default=0.005)
    parser.add_argument("--orientation-tolerance", type=positive, default=0.01)
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

#!/usr/bin/env python3
"""Print the current UR3 joint positions and TCP pose without moving the robot."""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


UR_JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


class UR3StateReader(Node):
    """Read one joint state and the latest TCP transform."""

    def __init__(self, base_frame: str, ee_frame: str) -> None:
        super().__init__("ur3_robot_state")
        self.base_frame = base_frame
        self.ee_frame = ee_frame
        self.joint_positions: dict[str, float] | None = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _joint_state_callback(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in UR_JOINT_NAMES):
            self.joint_positions = {
                name: positions[name] for name in UR_JOINT_NAMES
            }

    def read_and_print(self, timeout: float) -> bool:
        self.get_logger().info(
            f"Waiting for /joint_states and TF {self.base_frame} -> {self.ee_frame} ..."
        )
        deadline = time.monotonic() + timeout
        transform = None
        last_tf_error = "transform has not been received"

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.ee_frame,
                    rclpy.time.Time(),
                )
            except TransformException as exc:
                last_tf_error = str(exc)

            if self.joint_positions is not None and transform is not None:
                self._print_state(transform)
                return True

        if self.joint_positions is None:
            self.get_logger().error(
                "No complete UR3 joint state received on /joint_states."
            )
        if transform is None:
            self.get_logger().error(
                f"Could not look up TF {self.base_frame} -> {self.ee_frame}: "
                f"{last_tf_error}"
            )
        return False

    def _print_state(self, transform) -> None:
        print("\nCurrent joint positions:")
        for name in UR_JOINT_NAMES:
            radians = self.joint_positions[name]
            print(
                f"  {name:<21} {radians: .6f} rad "
                f"({math.degrees(radians): .3f} deg)"
            )

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        print(f"\nCurrent TCP pose ({self.base_frame} -> {self.ee_frame}):")
        print(
            "  position [m] : "
            f"x={translation.x:.6f}, y={translation.y:.6f}, z={translation.z:.6f}"
        )
        print(
            "  quaternion  : "
            f"x={rotation.x:.6f}, y={rotation.y:.6f}, "
            f"z={rotation.z:.6f}, w={rotation.w:.6f}"
        )


def positive(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Print the current UR3 joint positions and TCP pose."
    )
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--ee-frame", default="tool0")
    parser.add_argument("--timeout", type=positive, default=10.0)
    return parser.parse_args(argv)


def main(args=None) -> None:
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    cli_args = parse_args(rclpy.utilities.remove_ros_args(raw_args)[1:])
    rclpy.init(args=args)
    node = UR3StateReader(cli_args.base_frame, cli_args.ee_frame)
    try:
        success = node.read_and_print(cli_args.timeout)
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted by user.")
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

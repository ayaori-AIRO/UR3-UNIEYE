#!/usr/bin/env python3
"""Run an Ultralytics detection model on a ROS 2 image topic."""

import argparse
from pathlib import Path
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO


DEFAULT_MODEL = (
    Path.home() / "ros2_ws/src/ur3_moveit_examples/models/scissors_best.pt"
)


class YoloLiveDetection(Node):
    def __init__(self, cli: argparse.Namespace) -> None:
        super().__init__("yolo_live_detection")
        self.bridge = CvBridge()
        self.model = YOLO(cli.model)
        self.confidence = cli.confidence
        self.image_size = cli.image_size
        self.show_window = not cli.no_window
        self.frame_count = 0

        self.publisher = self.create_publisher(
            Image, "/yolo_detection/image", qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            Image, cli.image_topic, self._image_callback, qos_profile_sensor_data
        )

        self.get_logger().info(f"Model: {cli.model}")
        self.get_logger().info(f"Classes: {self.model.names}")
        self.get_logger().info(f"Input: {cli.image_topic}")
        self.get_logger().info("Output: /yolo_detection/image")
        if self.show_window:
            self.get_logger().info("Press q or ESC in the window to stop.")

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            result = self.model.predict(
                frame,
                conf=self.confidence,
                imgsz=self.image_size,
                device="cpu",
                verbose=False,
            )[0]
            annotated = result.plot()

            output = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            output.header = msg.header
            self.publisher.publish(output)

            self.frame_count += 1
            if self.frame_count % 30 == 0:
                detections = []
                if result.boxes is not None:
                    for class_id, confidence in zip(
                        result.boxes.cls.tolist(), result.boxes.conf.tolist()
                    ):
                        detections.append(
                            f"{result.names[int(class_id)]}={confidence:.2f}"
                        )
                summary = ", ".join(detections) if detections else "none"
                self.get_logger().info(f"Detections: {summary}")

            if self.show_window:
                cv2.imshow("D405 - YOLO detection", annotated)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    rclpy.shutdown()
        except Exception as exc:
            self.get_logger().error(f"Inference failed: {exc}")

    def destroy_node(self) -> None:
        cv2.destroyAllWindows()
        super().destroy_node()


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default=str(DEFAULT_MODEL))
    parser.add_argument("--image-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--confidence", type=float, default=0.50)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--no-window", action="store_true")
    cli = parser.parse_args(argv)
    cli.model = str(Path(cli.model).expanduser().resolve())
    if not Path(cli.model).is_file():
        parser.error(f"model file not found: {cli.model}")
    if not 0.0 <= cli.confidence <= 1.0:
        parser.error("--confidence must be between 0.0 and 1.0")
    return cli


def main(args=None) -> None:
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    cli = parse_args(rclpy.utilities.remove_ros_args(raw_args)[1:])
    rclpy.init(args=args)
    node = YoloLiveDetection(cli)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

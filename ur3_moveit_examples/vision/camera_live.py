"""Lightweight color viewer; no inference, TF, or robot commands."""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import time


def main():
    rclpy.init()
    node = Node('scenario_camera_live')
    bridge = CvBridge()
    latest = [None, 0.0]
    annotated = [None, 0.0]
    def receive(msg):
        latest[:] = [msg, time.monotonic()]
    node.create_subscription(Image, '/camera/camera/color/image_raw', receive,
                             qos_profile_sensor_data)
    def receive_annotated(msg):
        annotated[:] = [msg, time.monotonic()]
    node.create_subscription(Image, '/scissors/preview_image', receive_annotated,
                             qos_profile_sensor_data)
    try:
        cv2.namedWindow('D405 continuous live')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            if latest[0] is not None:
                frame = bridge.imgmsg_to_cv2(latest[0], 'bgr8').copy()
                label = ('STALE IMAGE' if time.monotonic()-latest[1] > 1
                         else 'LIVE - viewer only; q closes view, NOT robot stop')
                # Display the actual inference frame, never paste old boxes onto new RGB.
                if annotated[0] is not None and time.monotonic()-annotated[1] < 1.0:
                    frame = bridge.imgmsg_to_cv2(annotated[0], 'bgr8').copy()
                    label = 'DETECTION SNAPSHOT - returns to live after 1s'
                cv2.putText(frame, label, (10,frame.shape[0]-12), cv2.FONT_HERSHEY_SIMPLEX,
                            .5, (0,255,255), 1)
                cv2.imshow('D405 continuous live', frame)
            if cv2.waitKey(1) & 0xff in (ord('q'),27):
                break
            if cv2.getWindowProperty('D405 continuous live', cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

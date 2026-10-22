"""Read-only D405 dataset capture. Save unannotated full-resolution color PNGs."""
import argparse
from datetime import datetime
from pathlib import Path
import sys
import tempfile
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image


class DatasetCapture(Node):
    def __init__(self, topic, output):
        super().__init__('dataset_capture')
        self.bridge = CvBridge()
        self.frame = None
        self.received = 0.
        self.sequence = 0
        self.saved_sequence = -1
        self.count = 0
        self.last_save = 0.
        root = Path(output).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        self.output = Path(tempfile.mkdtemp(prefix=datetime.now().strftime('%Y%m%d_%H%M%S_'), dir=root))
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f'Input: {topic}; PNG output: {self.output}; S/SPACE: save, Q/ESC: quit')

    def on_image(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
            self.received = time.monotonic()
            self.sequence += 1
        except Exception as exc:
            self.get_logger().error(f'Image conversion failed: {exc}')

    def save(self):
        now = time.monotonic()
        if self.frame is None or now-self.received > 1.:
            self.get_logger().warning('No fresh image; not saved')
            return
        if self.sequence == self.saved_sequence or now-self.last_save < .4:
            return
        path = self.output / f'image_{self.count+1:06d}.png'
        # Encode only the original frame, never the resized/annotated preview.
        ok, encoded = cv2.imencode('.png', self.frame)
        if not ok:
            raise RuntimeError('PNG encoding failed')
        with path.open('xb') as stream:
            stream.write(encoded.tobytes())
        self.count += 1
        self.saved_sequence = self.sequence
        self.last_save = now
        self.get_logger().info(f'Saved #{self.count}: {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image-topic', default='/camera/camera/color/image_raw')
    parser.add_argument('--output-dir', default=str(Path.home()/'datasets/connector/images'))
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = None
    window = 'D405 dataset capture'
    try:
        node = DatasetCapture(args.image_topic, args.output_dir)
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window, 960, 540)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            preview = (node.frame.copy() if node.frame is not None
                       else np.zeros((480, 640, 3), dtype=np.uint8))
            stale = node.frame is None or time.monotonic()-node.received > 1.
            label = 'WAITING / STALE IMAGE' if stale else f'S / SPACE: save | Q: quit | Saved: {node.count}'
            cv2.putText(preview, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        .6, (0, 255, 255), 2)
            cv2.imshow(window, preview)
            key = cv2.waitKey(1) & 0xff
            if key in (27, ord('q'), ord('Q')) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key in (32, ord('s'), ord('S')):
                node.save()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

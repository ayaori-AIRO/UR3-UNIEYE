"""Read-only raw-image checkerboard preview; no robot control or TF writes."""

import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, Image


def object_points(columns, rows, square):
    points = np.zeros((columns * rows, 3), np.float32)
    points[:, :2] = np.mgrid[:columns, :rows].T.reshape(-1, 2) * square
    return points


class CheckerboardPreview(Node):
    def __init__(self, args):
        super().__init__('checkerboard_live')
        self.args = args
        self.bridge = CvBridge()
        self.info = None
        self.latest = None
        self.last_image = time.monotonic()
        self.done = False
        self.observation = None
        self.points = object_points(args.columns, args.rows, args.square_size)
        self.publisher = self.create_publisher(Image, '/checkerboard/image', 1)
        self.create_subscription(CameraInfo, args.info_topic, self.on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, args.image_topic, self.on_image,
                                 qos_profile_sensor_data)
        self.create_timer(0.2, self.process)
        self.create_timer(5.0, self.check_input)
        self.get_logger().info(
            f'Read-only: {args.columns}x{args.rows} inner corners, '
            f'{args.square_size} m squares. Waiting for {args.image_topic} '
            f'and {args.info_topic}. No TF or robot commands.')
        self.get_logger().warning(
            'Preview only: corner order/planar pose can flip. '
            'Not a validated hand-eye calibration. q/ESC closes preview.')

    def on_info(self, msg):
        self.info = msg

    def on_image(self, msg):
        self.latest = msg
        self.last_image = time.monotonic()

    def check_input(self):
        if time.monotonic() - self.last_image > 5:
            self.get_logger().warning('No recent image. Check camera launch and image topic.')
        if self.info is None:
            self.get_logger().warning('Waiting for CameraInfo; pose cannot be estimated.')

    def process(self):
        if self.latest is None:
            return
        msg, self.latest = self.latest, None
        self.observation = None
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            size = (self.args.columns, self.args.rows)
            found, corners = cv2.findChessboardCorners(
                gray, size, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
            label = 'Board not detected: show all 9x6 squares'
            if found:
                corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                          (cv2.TERM_CRITERIA_EPS |
                                           cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                cv2.drawChessboardCorners(frame, size, corners, found)
                label = self.draw_pose(frame, msg, corners)
            cv2.putText(frame, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 200, 255), 1, cv2.LINE_AA)
            output = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            output.header = msg.header
            self.publisher.publish(output)
            if not self.args.no_window:
                cv2.imshow('D405 checkerboard preview', frame)
                self.on_key(cv2.waitKey(1) & 0xff)
        except (cv2.error, ValueError, RuntimeError) as exc:
            self.get_logger().error(f'Preview failed: {exc}')

    def on_key(self, key):
        self.done = key in (27, ord('q'))

    def draw_pose(self, frame, msg, corners):
        info = self.info
        if info is None:
            return 'Corners found; waiting for CameraInfo'
        if (info.width, info.height) != (msg.width, msg.height):
            return 'CameraInfo/image size mismatch'
        if not msg.header.frame_id or info.header.frame_id != msg.header.frame_id:
            return 'CameraInfo/image frame mismatch'
        if info.distortion_model not in ('plumb_bob', 'rational_polynomial'):
            return 'Unsupported distortion model (raw image required)'
        if info.binning_x > 1 or info.binning_y > 1 or info.roi.width or info.roi.height:
            return 'ROI/binning unsupported; use full raw image'
        k = np.array(info.k, dtype=np.float64).reshape(3, 3)
        d = np.array(info.d, dtype=np.float64)
        if not np.isfinite(k).all() or not np.isfinite(d).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
            return 'Invalid camera intrinsics'
        ok, rotation, translation = cv2.solvePnP(self.points, corners, k, d)
        if not ok or not np.isfinite(translation).all() or not np.isfinite(rotation).all():
            return 'Pose estimation failed'
        r, _ = cv2.Rodrigues(rotation)
        if np.any((self.points @ r.T + translation.reshape(3))[:, 2] <= 0):
            return 'Invalid pose behind camera'
        projected, _ = cv2.projectPoints(self.points, rotation, translation, k, d)
        error = np.sqrt(np.mean(np.sum((projected - corners) ** 2, axis=2)))
        self.observation = (msg, info, corners.copy(), rotation.copy(),
                            translation.copy(), float(error))
        cv2.drawFrameAxes(frame, k, d, rotation, translation, self.args.square_size * 2)
        x, y, z = translation.ravel()
        return f'PREVIEW xyz[m]={x:.3f},{y:.3f},{z:.3f} reproj={error:.2f}px'


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--columns', type=int, default=8, help='Internal corner columns')
    parser.add_argument('--rows', type=int, default=5, help='Internal corner rows')
    parser.add_argument('--square-size', type=float, default=0.020, help='Measured square size in metres')
    parser.add_argument('--image-topic', default='/camera/camera/color/image_raw')
    parser.add_argument('--info-topic', default='/camera/camera/color/camera_info')
    parser.add_argument('--no-window', action='store_true')
    cli = parser.parse_args(remove_ros_args(args=args)[1:])
    if cli.columns < 3 or cli.rows < 3 or not np.isfinite(cli.square_size) or cli.square_size <= 0:
        parser.error('Need at least 3x3 inner corners and a finite positive square size')
    if not cli.no_window and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        parser.error('No display available; use --no-window and view /checkerboard/image')
    rclpy.init(args=args)
    node = CheckerboardPreview(cli)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if not cli.no_window:
            cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

"""Read-only, operator-selected scissors surface point from aligned RGB-D."""

import argparse
from collections import deque
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

from ur3_moveit_examples.vision.yolo_live_detection import DEFAULT_MODEL


def seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def surface_point(depth, encoding, u, v, info):
    """Require a small, mostly valid, locally flat surface around clicked pixel."""
    if encoding not in ('16UC1', '32FC1'):
        raise ValueError('Depth must be 16UC1 (mm) or 32FC1 (m)')
    if not (2 <= u < depth.shape[1]-2 and 2 <= v < depth.shape[0]-2):
        raise ValueError('Click farther from image edge')
    patch = depth[v-2:v+3, u-2:u+3].astype(float)
    if encoding == '16UC1':
        patch *= 0.001
    valid = np.isfinite(patch) & (patch >= 0.07) & (patch <= 0.50)
    if not valid[2, 2] or valid.sum() < 20:
        raise ValueError('Insufficient valid depth (required range 0.07–0.50 m)')
    values = patch[valid]
    if np.ptp(values) > 0.006:
        raise ValueError('Depth edge/discontinuity: select a flat scissors surface')
    k = np.asarray(info.k, dtype=float).reshape(3, 3)
    d = np.asarray(info.d, dtype=float)
    if (info.distortion_model not in ('plumb_bob', 'rational_polynomial')
            or not np.isfinite(k).all() or not np.isfinite(d).all()
            or k[0, 0] <= 0 or k[1, 1] <= 0):
        raise ValueError('Unsupported or invalid CameraInfo')
    if info.binning_x > 1 or info.binning_y > 1 or any((
            info.roi.x_offset, info.roi.y_offset, info.roi.width, info.roi.height)):
        raise ValueError('Binned/cropped CameraInfo is not supported')
    ray = cv2.undistortPoints(np.array([[[u, v]]], dtype=float), k, d).ravel()
    return np.array([ray[0], ray[1], 1.0]) * np.median(values)


def transform_point(point, transform):
    q = transform.rotation
    quat = np.array([q.x, q.y, q.z, q.w])
    if not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1) > 0.01:
        raise ValueError('Invalid TF quaternion')
    quat /= np.linalg.norm(quat)
    xyz, w = quat[:3], quat[3]
    rotated = point + 2*np.cross(xyz, np.cross(xyz, point) + w*point)
    t = transform.translation
    result = rotated + np.array([t.x, t.y, t.z])
    if not np.isfinite(result).all():
        raise ValueError('Non-finite transformed point')
    return result


def automatic_pixel(frame, box):
    """GrabCut foreground heuristic, not a semantic scissors segmentation model."""
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = map(int, box)
    x1, y1 = max(1, x1), max(1, y1)
    x2, y2 = min(w-1, x2), min(h-1, y2)
    if x2-x1 < 10 or y2-y1 < 10:
        raise ValueError('Automatic box too small')
    mask = np.zeros((h, w), np.uint8)
    cv2.grabCut(frame, mask, (x1,y1,x2-x1,y2-y1),
                np.zeros((1,65), np.float64), np.zeros((1,65), np.float64),
                3, cv2.GC_INIT_WITH_RECT)
    foreground = np.uint8((mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD))
    area = foreground.sum() / ((x2-x1)*(y2-y1))
    if not 0.03 < area < 0.85:
        raise ValueError('Ambiguous foreground area; no automatic point')
    distance = cv2.distanceTransform(foreground, cv2.DIST_L2, 5)
    v, u = np.unravel_index(np.argmax(distance), distance.shape)
    if distance[v,u] < 4:
        raise ValueError('Foreground too thin for a depth patch')
    return int(u), int(v)


def stable_auto_point(history, now):
    """8 recent attempts, >=5 valid samples, >=2 scores >=0.5, 8mm radius."""
    samples = [s for s in history if s is not None and now-s[0] <= 10]
    if len(samples) < 5 or sum(s[1] >= 0.5 for s in samples) < 2:
        return None
    points = np.asarray([s[2] for s in samples])
    centre = np.median(points, axis=0)
    if np.max(np.linalg.norm(points-centre, axis=1)) > 0.008:
        return None
    return centre


class ScissorsPosition(Node):
    def __init__(self, cli):
        super().__init__('scissors_position')
        from ultralytics import YOLO
        self.model = YOLO(cli.model)
        self.cli = cli
        self.bridge = CvBridge()
        self.rgb = None
        self.info = None
        self.depths = deque(maxlen=30)
        self.snapshot = None
        self.auto_history = deque(maxlen=8)
        self.last_auto_stamp = None
        self.next_auto = 0.0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        prefix = '/scissors/auto_preview' if cli.auto else '/scissors/candidate'
        self.point_pub = self.create_publisher(PointStamped, prefix + '_point', 10)
        self.marker_pub = self.create_publisher(Marker, prefix + '_marker', 10)
        self.subs = [
            self.create_subscription(Image, cli.image_topic, self.on_rgb, qos_profile_sensor_data),
            self.create_subscription(Image, cli.depth_topic, self.depths.append, qos_profile_sensor_data),
            self.create_subscription(CameraInfo, cli.info_topic, self.on_info, qos_profile_sensor_data),
        ]
        self.image_pub = self.create_publisher(
            Image, '/scissors/preview_image', qos_profile_sensor_data)
        if not cli.no_window:
            cv2.namedWindow('Scissors position')
            cv2.setMouseCallback('Scissors position', self.on_click)
        self.timer = self.create_timer(0.1, self.preview)
        self.get_logger().info(
            'READ ONLY. Keep robot/object stopped. s: detect and freeze; '
            'click scissors surface inside box; r: live; q/ESC: quit. '
            'Candidate only, NOT a grasp target. Snapshots expire after 10 seconds.')
        if cli.auto:
            self.get_logger().warning(
                'AUTO PREVIEW ONLY: no clicks or motion-topic output. GrabCut is a '
                'heuristic; check against the object. '
                f'policy={cli.auto_policy}, confidence={cli.confidence}. '
                'Keep robot and object stationary.')

    def on_rgb(self, msg):
        self.rgb = msg

    def on_info(self, msg):
        self.info = msg

    def clear(self):
        self.snapshot = None
        marker = Marker()
        marker.ns = 'scissors_candidate'
        marker.id = 0
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)

    def capture(self):
        self.clear()
        rgb, info = self.rgb, self.info
        if rgb is None or info is None or not self.depths:
            raise ValueError('Waiting for color, CameraInfo and aligned depth')
        age = self.get_clock().now().nanoseconds*1e-9-seconds(rgb)
        if seconds(rgb) <= 0 or not 0 <= age <= 0.5:
            raise ValueError('Color timestamp is stale or in the future')
        depth = min(self.depths, key=lambda m: abs(seconds(m)-seconds(rgb)))
        if abs(seconds(depth)-seconds(rgb)) > 0.03:
            raise ValueError('RGB/depth timestamps differ by more than 30 ms')
        if abs(seconds(info)-seconds(rgb)) > 1.0:
            raise ValueError('CameraInfo is stale')
        if not rgb.header.frame_id or not (
                rgb.header.frame_id == depth.header.frame_id == info.header.frame_id):
            raise ValueError('Color/aligned-depth/CameraInfo frames must match')
        if not ((rgb.width, rgb.height) == (depth.width, depth.height)
                == (info.width, info.height)):
            raise ValueError('RGB/depth/CameraInfo resolutions differ')
        # Exact exposure timestamp, never latest TF. Resolve before slow inference.
        tf = self.buffer.lookup_transform(
            'base_link', rgb.header.frame_id, Time.from_msg(rgb.header.stamp))
        frame = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
        result = self.model.predict(frame, conf=(0.30 if self.cli.auto and
                                    self.cli.auto_policy == 'stable' else self.cli.confidence),
                                    imgsz=640, device='cpu', verbose=False)[0]
        boxes = []
        scores = []
        if result.boxes is not None:
            for box, cls, score in zip(result.boxes.xyxy.tolist(), result.boxes.cls.tolist(),
                                       result.boxes.conf.tolist()):
                if str(result.names[int(cls)]).lower() == 'scissors':
                    boxes.append(box)
                    scores.append(score)
        if not boxes:
            raise ValueError('No scissors detected')
        drawn = frame.copy()
        for x1, y1, x2, y2 in boxes:
            cv2.rectangle(drawn, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
        self.publish_image(drawn, rgb.header)
        cv2.putText(drawn, 'FROZEN: click scissors surface; r = live',
                    (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        self.snapshot = (drawn, self.bridge.imgmsg_to_cv2(depth, 'passthrough'),
                         depth.encoding, info, rgb.header, tf.transform, boxes,
                         time.monotonic())
        if self.cli.auto:
            if len(boxes) != 1:
                self.auto_history.clear()
                raise ValueError('Multiple scissors: automatic selection rejected')
            u, v = automatic_pixel(frame, boxes[0])
            camera = surface_point(self.snapshot[1], depth.encoding, u, v, info)
            base = transform_point(camera, tf.transform)
            # Reject stale inference, rather than publishing an old observation.
            if self.get_clock().now().nanoseconds*1e-9-seconds(rgb) > 3:
                raise ValueError('Automatic inference exceeded 3s image age')
            now = time.monotonic()
            if self.auto_history and self.auto_history[-1] is not None:
                if np.linalg.norm(base-self.auto_history[-1][2]) > 0.02:
                    self.auto_history.clear()
            self.auto_history.append((now, scores[0], base))
            stable = (base if self.cli.auto_policy == 'first-valid'
                      else stable_auto_point(self.auto_history, now))
            cv2.rectangle(drawn, (0,0), (drawn.shape[1],35), (0,0,0), -1)
            cv2.putText(drawn, f'AUTO conf={scores[0]:.2f} ' +
                        ('VALID PREVIEW' if stable is not None else 'ACCUMULATING'),
                        (10,25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0,255,255), 2)
            cv2.circle(drawn, (u,v), 5, (0,0,255), 2)
            self.publish_image(drawn, rgb.header)
            if stable is not None:
                # Publish newest measured point, not a median with a mismatched stamp.
                self.publish_point(base, rgb.header)
                self.get_logger().info(f'AUTO PREVIEW base_link [m]: {base.tolist()}')

    def publish_image(self, frame, header):
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header = header
        self.image_pub.publish(msg)

    def publish_point(self, base, header):
        msg = PointStamped()
        msg.header.stamp = header.stamp
        msg.header.frame_id = 'base_link'
        msg.point.x, msg.point.y, msg.point.z = map(float, base)
        self.point_pub.publish(msg)
        marker = Marker()
        marker.header = msg.header
        marker.ns = 'scissors_candidate'
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = msg.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.01
        marker.color.r = marker.color.a = 1.0
        marker.color.g = 0.7
        marker.lifetime = Duration(seconds=3.0).to_msg()
        self.marker_pub.publish(marker)

    def on_click(self, event, u, v, flags, param):
        if self.cli.auto:
            return
        if event != cv2.EVENT_LBUTTONDOWN or self.snapshot is None:
            return
        try:
            frame, depth, encoding, info, header, tf, boxes, created = self.snapshot
            if time.monotonic()-created > 10:
                raise ValueError('Snapshot expired; press r then s')
            if not any(x1+2 <= u <= x2-2 and y1+2 <= v <= y2-2
                       for x1,y1,x2,y2 in boxes):
                raise ValueError('Click inside a scissors box, on the object itself')
            camera = surface_point(depth, encoding, u, v, info)
            base = transform_point(camera, tf)
            msg = PointStamped()
            msg.header.stamp = header.stamp
            msg.header.frame_id = 'base_link'
            msg.point.x, msg.point.y, msg.point.z = map(float, base)
            self.point_pub.publish(msg)
            marker = Marker()
            marker.header = msg.header
            marker.ns = 'scissors_candidate'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = msg.point
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.01
            marker.color.r = marker.color.a = 1.0
            marker.color.g = 0.7
            marker.lifetime = Duration(seconds=3.0).to_msg()
            self.marker_pub.publish(marker)
            cv2.circle(frame, (u,v), 5, (0,0,255), 2)
            self.get_logger().info(f'CANDIDATE base_link [m]: {base.tolist()}; '
                                   f'camera [m]: {camera.tolist()}; NOT a motion target')
        except Exception as exc:
            self.clear()
            self.get_logger().warning(f'Point rejected: {exc}')

    def preview(self):
        try:
            if self.cli.auto and time.monotonic() >= self.next_auto and self.rgb is not None:
                stamp = (self.rgb.header.stamp.sec, self.rgb.header.stamp.nanosec)
                if stamp != self.last_auto_stamp:
                    self.last_auto_stamp = stamp
                    self.next_auto = time.monotonic() + 0.5
                    self.capture()
            if self.snapshot is not None and time.monotonic()-self.snapshot[-1] > 10:
                self.clear()
            if self.cli.no_window:
                return
            if self.snapshot is not None:
                cv2.imshow('Scissors position', self.snapshot[0])
            elif self.rgb is not None:
                cv2.imshow('Scissors position', self.bridge.imgmsg_to_cv2(self.rgb, 'bgr8'))
            key = cv2.waitKey(1) & 0xff
            if key in (ord('q'), 27):
                rclpy.shutdown()
            elif key == ord('r'):
                self.auto_history.clear()
                self.clear()
            elif key == ord('s') and not self.cli.auto:
                self.capture()
        except Exception as exc:
            if self.cli.auto:
                self.auto_history.append(None)
                if not self.cli.no_window and cv2.waitKey(1) & 0xff in (ord('q'), 27):
                    rclpy.shutdown()
            self.clear()
            self.get_logger().warning(f'Snapshot rejected: {exc}')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default=str(DEFAULT_MODEL))
    parser.add_argument('--no-window', action='store_true',
                        help='publish preview images instead of opening a detector window (auto only)')
    parser.add_argument('--auto', action='store_true',
                        help='automatic point preview; retries until stopped, no motion')
    parser.add_argument('--auto-policy', choices=('first-valid', 'stable'),
                        default='first-valid', help='first-valid: no multi-frame voting; stable: legacy voting')
    parser.add_argument('--confidence', type=float, default=0.4)
    parser.add_argument('--image-topic', default='/camera/camera/color/image_raw')
    parser.add_argument('--depth-topic', default='/camera/camera/aligned_depth_to_color/image_raw')
    parser.add_argument('--info-topic', default='/camera/camera/color/camera_info')
    cli = parser.parse_args(rclpy.utilities.remove_ros_args(
        sys.argv if args is None else [sys.argv[0], *args])[1:])
    if not 0 < cli.confidence <= 1:
        parser.error('confidence must be in (0,1]')
    if cli.no_window and not cli.auto:
        parser.error('--no-window requires --auto; manual mode needs clicks')
    rclpy.init(args=args)
    node = None
    try:
        node = ScissorsPosition(cli)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if not cli.no_window:
            cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

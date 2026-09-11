"""Manually save timestamped hand-eye candidates; never commands the robot."""
import argparse
from collections import deque
from datetime import datetime
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from .checkerboard_live import CheckerboardPreview


JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stationary(history, image_time):
    samples = [(t, p) for t, p in history if image_time - 1.2 <= t <= image_time + .1]
    if len(samples) < 5 or samples[-1][0] - samples[0][0] < 1.0:
        raise ValueError('Need at least one second of complete joint states while stopped')
    if abs(samples[-1][0] - image_time) > .15:
        raise ValueError('Joint states are not close to image timestamp')
    if max(np.diff([t for t, _ in samples])) > .2:
        raise ValueError('Joint state stream has gaps')
    if np.max(np.ptp([p for _, p in samples], axis=0)) > .001:
        raise ValueError('Robot is moving: joint excursion exceeds 0.001 rad')
    return samples


class Capture(CheckerboardPreview):
    def __init__(self, args):
        super().__init__(args)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.history = deque(maxlen=3000)
        self.saved_stamp = None
        self.create_subscription(JointState, '/joint_states', self.joints,
                                 qos_profile_sensor_data)
        self.create_service(Trigger, '/handeye_capture/save', self.save_service)
        self.directory = Path(args.output).expanduser() / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.directory.mkdir(parents=True, exist_ok=False)
        self.get_logger().info(f'Press s to save; output: {self.directory}')
        self.get_logger().warning('Keep board fixed. Check origin/axes before EVERY save. '
                                  'Samples are candidates, not calibration results.')

    def joints(self, msg):
        if len(msg.name) != len(msg.position):
            return
        values = dict(zip(msg.name, msg.position))
        if not all(n in values for n in JOINTS):
            return
        p = [values[n] for n in JOINTS]
        t = seconds(msg.header.stamp)
        if not np.isfinite(p).all() or t <= 0:
            return
        if self.history and t <= self.history[-1][0]:
            self.history.clear()
        self.history.append((t, p))

    def on_key(self, key):
        super().on_key(key)
        if key == ord('s'):
            ok, message = self.save()
            if ok:
                self.get_logger().info(message)
            else:
                self.get_logger().warning(message)

    def save_service(self, request, response):
        response.success, response.message = self.save()
        return response

    def save(self):
        try:
            if self.observation is None:
                raise ValueError('No valid checkerboard pose')
            msg, info, corners, rotation, translation, error = self.observation
            stamp = seconds(msg.header.stamp)
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if stamp <= 0 or not 0 <= age <= .5 or time.monotonic() - self.last_image > .5:
                raise ValueError('Image is stale or timestamp clock differs')
            if stamp == self.saved_stamp:
                raise ValueError('This frame was already saved')
            if not np.isfinite(error) or error > .5:
                raise ValueError('Reprojection RMS must be <= 0.5 px')
            states = stationary(self.history, stamp)
            transform = self.tf.lookup_transform('base_link', 'tool0', Time.from_msg(msg.header.stamp))
            t, q = transform.transform.translation, transform.transform.rotation
            record = {
                'schema_version': 1, 'status': 'candidate_not_validated',
                'image_stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
                'camera_frame': msg.header.frame_id, 'base_frame': 'base_link', 'tool_frame': 'tool0',
                'columns': self.args.columns, 'rows': self.args.rows,
                'square_size_m': self.args.square_size,
                'camera_matrix': list(info.k), 'distortion': list(info.d),
                'distortion_model': info.distortion_model,
                'width': msg.width, 'height': msg.height,
                'corners_px': corners.reshape(-1, 2).tolist(),
                'camera_T_board': {'rvec': rotation.ravel().tolist(), 'translation_m': translation.ravel().tolist()},
                'base_T_tool': {'translation_m': [t.x, t.y, t.z], 'quaternion_xyzw': [q.x, q.y, q.z, q.w]},
                'reprojection_rms_px': error, 'joint_names': JOINTS,
                'stationary_joint_samples': states,
            }
            # Exclusive per-sample directory prevents overwriting earlier data.
            target = self.directory / f'{msg.header.stamp.sec}_{msg.header.stamp.nanosec}'
            target.mkdir(exist_ok=False)
            if not cv2.imwrite(str(target / 'image.png'), self.bridge.imgmsg_to_cv2(msg, 'bgr8')):
                raise OSError('Image write failed; incomplete sample directory retained')
            with (target / 'sample.json').open('x') as stream:
                json.dump(record, stream, indent=2, allow_nan=False)
            self.saved_stamp = stamp
            return True, f'Saved candidate: {target}'
        except (ValueError, OSError, TransformException, cv2.error) as exc:
            return False, f'Not saved: {exc}'


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default=str(Path.home() / 'handeye_samples'))
    parser.add_argument('--no-window', action='store_true')
    cli = parser.parse_args(remove_ros_args(args=args)[1:])
    cli.columns, cli.rows, cli.square_size = 8, 5, .020
    cli.image_topic = '/camera/camera/color/image_raw'
    cli.info_topic = '/camera/camera/color/camera_info'
    import os
    if not cli.no_window and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        parser.error('No display: use --no-window')
    rclpy.init(args=args)
    node = None
    try:
        node = Capture(cli)
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if not cli.no_window:
            cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

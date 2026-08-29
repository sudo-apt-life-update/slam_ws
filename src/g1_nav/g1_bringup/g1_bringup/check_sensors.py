#!/usr/bin/env python3
"""One-shot sensor and TF diagnostic for the G1.

Answers "are my sensors and TF correct?" with a pass/fail table rather than a
wall of ros2 topic echo. Run it with the sensor stack up:

    ros2 launch g1_bringup sensors.launch.py
    ros2 run g1_bringup check_sensors

Exits 0 if every check passes, 1 otherwise, so it can gate a script.

What it cannot check is noted at the end: the physical tests (optical frame
orientation, extrinsic accuracy, gravity alignment) need a human with a tape
measure and RViz.
"""

import argparse
import math
import sys
import time
from collections import defaultdict

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Imu, JointState
from tf2_ros import Buffer, TransformListener

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - cv_bridge is a hard dep in practice
    CvBridge = None

from sensor_msgs.msg import Image

# --- expectations -----------------------------------------------------------
# Rates are floors, not targets. Camera sits at ~7 Hz until image_server
# compresses depth; raise CAMERA_MIN_HZ to 25 once that lands.
IMU_MIN_HZ = 150.0
JOINT_MIN_HZ = 50.0
CAMERA_MIN_HZ = 5.0
TF_MIN_HZ = 50.0

STATIONARY_GRAVITY = (9.5, 10.1)   # m/s^2, |accel| with the robot still
STATIONARY_GYRO_MAX = 0.05         # rad/s, bias while still
SYNC_MAX_MS = 5.0                  # |t_colour - t_depth|
AGE_MAX_MS = 500.0                 # how stale the newest frame may be
# A few ms of negative age is jitter and rounding; anything more means the
# publisher's clock is ahead of ours, which is a real fault, not staleness.
AGE_MIN_MS = -5.0
DEPTH_MIN_VALID_FRACTION = 0.30
DEPTH_RANGE_M = (0.1, 10.0)

NUM_JOINTS = 29

# The chain the mapping stack will walk. Frames that do not exist yet are
# reported as such rather than crashing -- the URDF work is still pending.
TF_CHAIN = [
    ('base_footprint', 'base_link'),
    ('base_link', 'pelvis'),
    ('pelvis', 'torso_link'),
    ('torso_link', 'd435_link'),
    ('d435_link', 'camera_color_optical_frame'),
    ('base_link', 'camera_color_optical_frame'),
]

GREEN, RED, YELLOW, DIM, RESET = (
    '\033[32m', '\033[31m', '\033[33m', '\033[2m', '\033[0m')


class Result:
    def __init__(self):
        self.rows = []

    def add(self, group, name, status, detail=''):
        self.rows.append((group, name, status, detail))

    def ok(self, group, name, detail=''):
        self.add(group, name, 'PASS', detail)

    def fail(self, group, name, detail=''):
        self.add(group, name, 'FAIL', detail)

    def warn(self, group, name, detail=''):
        self.add(group, name, 'WARN', detail)

    def render(self):
        width = max(len(n) for _, n, _, _ in self.rows) + 2
        current = None
        for group, name, status, detail in self.rows:
            if group != current:
                print(f'\n{DIM}{group}{RESET}')
                current = group
            colour = {'PASS': GREEN, 'FAIL': RED, 'WARN': YELLOW}[status]
            print(f'  {colour}{status:4s}{RESET}  {name:<{width}} {detail}')

        failed = sum(1 for r in self.rows if r[2] == 'FAIL')
        warned = sum(1 for r in self.rows if r[2] == 'WARN')
        passed = sum(1 for r in self.rows if r[2] == 'PASS')
        print(f'\n{passed} passed, {failed} failed, {warned} warnings')
        return failed


class SensorCheck(Node):
    def __init__(self, duration):
        super().__init__('check_sensors')
        self.duration = duration
        self.stamps = defaultdict(list)
        self.latest = {}

        # Best-effort for everything: it is the compatible subscriber against
        # both reliable and best-effort publishers, whereas a reliable
        # subscriber silently receives nothing from a best-effort publisher.
        # One subscription per topic, or the measured rate doubles.
        qos = QoSProfile(depth=20,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        self._sub(Imu, '/imu/data', qos)
        self._sub(Imu, '/imu_torso/data', qos)
        self._sub(JointState, '/joint_states', qos)
        self._sub(Image, '/camera/color/image_raw', qos)
        self._sub(Image, '/camera/aligned_depth_to_color/image_raw', qos)
        self._sub(CameraInfo, '/camera/color/camera_info', qos)
        self._sub(CameraInfo,
                  '/camera/aligned_depth_to_color/camera_info', qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_stamps = []
        from tf2_msgs.msg import TFMessage
        self.create_subscription(
            TFMessage, '/tf',
            lambda m: self.tf_stamps.append(time.time()), 100)

    def _sub(self, msg_type, topic, qos):
        # Subscribe with both reliabilities where it matters: a best-effort
        # subscriber cannot receive from a reliable publisher's perspective is
        # fine, but a reliable subscriber gets nothing from a best-effort
        # publisher. Duplicates are de-duplicated by stamp below.
        self.create_subscription(
            msg_type, topic,
            lambda m, t=topic: self._on_msg(t, m), qos)

    def _on_msg(self, topic, msg):
        self.stamps[topic].append(time.time())
        self.latest[topic] = msg

    def collect(self):
        end = time.time() + self.duration
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def rate(self, topic):
        n = len(self.stamps.get(topic, []))
        return n / self.duration if n else 0.0


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def check_rates(node, r):
    group = 'Rates'
    for topic, floor in (
        ('/imu/data', IMU_MIN_HZ),
        ('/imu_torso/data', IMU_MIN_HZ),
        ('/joint_states', JOINT_MIN_HZ),
        ('/camera/color/image_raw', CAMERA_MIN_HZ),
        ('/camera/aligned_depth_to_color/image_raw', CAMERA_MIN_HZ),
        ('/camera/color/camera_info', CAMERA_MIN_HZ),
    ):
        hz = node.rate(topic)
        detail = f'{hz:7.1f} Hz  (need >= {floor:.0f})'
        if hz == 0.0:
            r.fail(group, topic, 'no publisher')
        elif hz < floor:
            r.fail(group, topic, detail)
        else:
            r.ok(group, topic, detail)

    tf_hz = len(node.tf_stamps) / node.duration
    detail = f'{tf_hz:7.1f} Hz  (need >= {TF_MIN_HZ:.0f})'
    if tf_hz == 0.0:
        r.fail(group, '/tf', 'nothing publishing TF')
    elif tf_hz < TF_MIN_HZ:
        r.fail(group, '/tf', detail +
               '  -- raise robot_state_publisher publish_frequency')
    else:
        r.ok(group, '/tf', detail)


def check_imu(node, r):
    group = 'IMU'
    msg = node.latest.get('/imu/data')
    if msg is None:
        r.fail(group, 'imu present', 'no /imu/data received')
        return

    r.ok(group, 'frame_id', msg.header.frame_id)

    q = msg.orientation
    norm = math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
    if abs(norm - 1.0) < 1e-3:
        r.ok(group, 'quaternion normalised', f'|q| = {norm:.6f}')
    else:
        r.fail(group, 'quaternion normalised',
               f'|q| = {norm:.6f} -- check the (w,x,y,z) -> (x,y,z,w) reorder')

    a = msg.linear_acceleration
    mag = math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2)
    lo, hi = STATIONARY_GRAVITY
    if lo <= mag <= hi:
        r.ok(group, 'gravity magnitude', f'|a| = {mag:.3f} m/s^2')
    else:
        r.fail(group, 'gravity magnitude',
               f'|a| = {mag:.3f} m/s^2, expected {lo}-{hi} (robot must be still)')

    g = msg.angular_velocity
    gyro = math.sqrt(g.x ** 2 + g.y ** 2 + g.z ** 2)
    if gyro < STATIONARY_GYRO_MAX:
        r.ok(group, 'gyro bias (stationary)', f'|w| = {gyro:.5f} rad/s')
    else:
        r.warn(group, 'gyro bias (stationary)',
               f'|w| = {gyro:.5f} rad/s -- is the robot moving?')

    if any(msg.orientation_covariance):
        r.ok(group, 'covariances populated',
             f'orient[0] = {msg.orientation_covariance[0]:g}')
    else:
        r.fail(group, 'covariances populated',
               'all zero -- robot_localization reads these')
    if msg.orientation_covariance[0] < 0:
        r.fail(group, 'orientation usable',
               'covariance[0] = -1 means "no orientation"')


def check_joints(node, r):
    group = 'Joints'
    msg = node.latest.get('/joint_states')
    if msg is None:
        r.fail(group, 'joint_states present', 'nothing received')
        return

    if len(msg.name) == NUM_JOINTS:
        r.ok(group, 'joint count', f'{len(msg.name)}')
    else:
        r.fail(group, 'joint count', f'{len(msg.name)}, expected {NUM_JOINTS}')

    for field in ('position', 'velocity', 'effort'):
        values = list(getattr(msg, field))
        if len(values) != len(msg.name):
            r.fail(group, f'{field} populated',
                   f'{len(values)} values for {len(msg.name)} joints')
        elif all(math.isfinite(v) for v in values):
            r.ok(group, f'{field} populated', f'{len(values)} finite values')
        else:
            r.fail(group, f'{field} populated', 'contains NaN or inf')


def check_camera(node, r):
    group = 'Camera'
    colour = node.latest.get('/camera/color/image_raw')
    depth = node.latest.get('/camera/aligned_depth_to_color/image_raw')
    info = node.latest.get('/camera/color/camera_info')

    if colour is None or depth is None or info is None:
        missing = [n for n, m in (('colour', colour), ('depth', depth),
                                  ('camera_info', info)) if m is None]
        r.fail(group, 'topics present', f'missing {", ".join(missing)}')
        return

    r.ok(group, 'colour encoding',
         f'{colour.width}x{colour.height} {colour.encoding}')
    if depth.encoding != '16UC1':
        r.fail(group, 'depth encoding',
               f'{depth.encoding}, expected 16UC1 (millimetres)')
    else:
        r.ok(group, 'depth encoding',
             f'{depth.width}x{depth.height} {depth.encoding}')

    if (colour.width, colour.height) == (depth.width, depth.height):
        r.ok(group, 'depth aligned to colour', 'same resolution')
    else:
        r.fail(group, 'depth aligned to colour',
               f'colour {colour.width}x{colour.height} vs '
               f'depth {depth.width}x{depth.height}')

    frames = {colour.header.frame_id, depth.header.frame_id,
              info.header.frame_id}
    if len(frames) == 1:
        r.ok(group, 'frames consistent', frames.pop())
    else:
        r.fail(group, 'frames consistent', f'differing: {sorted(frames)}')

    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    if fx > 0 and fy > 0:
        r.ok(group, 'focal lengths non-zero', f'fx={fx:.1f} fy={fy:.1f}')
    else:
        r.fail(group, 'focal lengths non-zero',
               f'fx={fx} fy={fy} -- CameraInfo not populated')
    if fx > 0 and abs(fx - fy) / fx < 0.2:
        r.ok(group, 'fx/fy within 20%', f'{abs(fx - fy) / fx * 100:.2f}%')
    else:
        r.fail(group, 'fx/fy within 20%', f'fx={fx:.1f} fy={fy:.1f}')

    if info.width == colour.width and info.height == colour.height:
        r.ok(group, 'info dims match image', f'{info.width}x{info.height}')
    else:
        r.fail(group, 'info dims match image',
               f'info {info.width}x{info.height} vs '
               f'image {colour.width}x{colour.height}')

    for label, value, extent in (('cx', cx, colour.width),
                                 ('cy', cy, colour.height)):
        if abs(value - extent / 2) < extent * 0.15:
            r.ok(group, f'{label} near centre',
                 f'{value:.1f} vs centre {extent / 2:.0f}')
        else:
            r.warn(group, f'{label} near centre',
                   f'{value:.1f} vs centre {extent / 2:.0f}')

    # timestamp sync
    dt_ms = abs(stamp_seconds(colour.header) - stamp_seconds(depth.header)) * 1e3
    if dt_ms < SYNC_MAX_MS:
        r.ok(group, 'colour/depth sync', f'{dt_ms:.3f} ms')
    else:
        r.fail(group, 'colour/depth sync',
               f'{dt_ms:.1f} ms (need < {SYNC_MAX_MS}) -- one stamp per frame?')

    age_ms = (time.time() - stamp_seconds(colour.header)) * 1e3
    if age_ms < AGE_MIN_MS:
        # A frame cannot be captured in the future. This is the signature of
        # the camera being stamped by the robot's clock while the IMU and
        # joints are stamped by this PC's -- the two streams are then
        # misaligned by the skew, which silently corrupts fusion.
        r.fail(group, 'frame age',
               f'{age_ms:.0f} ms -- stamped in the FUTURE, so the robot and '
               f'this PC disagree about the time. Compare: '
               f'ssh unitree@192.168.123.164 date +%s.%N   vs   date +%s.%N')
    elif age_ms < AGE_MAX_MS:
        r.ok(group, 'frame age', f'{age_ms:.0f} ms')
    else:
        r.fail(group, 'frame age',
               f'{age_ms:.0f} ms -- clock skew, or the stream stalled')

    # depth content
    if CvBridge is None:
        r.warn(group, 'depth content', 'cv_bridge unavailable, skipped')
        return
    try:
        image = CvBridge().imgmsg_to_cv2(depth, '16UC1')
    except Exception as exc:  # noqa: BLE001
        r.fail(group, 'depth content', f'could not convert: {exc}')
        return

    valid = image[image > 0]
    fraction = valid.size / image.size if image.size else 0.0
    if fraction >= DEPTH_MIN_VALID_FRACTION:
        r.ok(group, 'depth valid pixels', f'{fraction * 100:.1f}%')
    else:
        r.fail(group, 'depth valid pixels',
               f'{fraction * 100:.1f}% (need >= '
               f'{DEPTH_MIN_VALID_FRACTION * 100:.0f}%)')

    if valid.size:
        lo, hi = valid.min() / 1000.0, valid.max() / 1000.0
        rlo, rhi = DEPTH_RANGE_M
        if rlo <= lo and hi <= rhi:
            r.ok(group, 'depth range', f'{lo:.2f} .. {hi:.2f} m')
        else:
            r.warn(group, 'depth range',
                   f'{lo:.2f} .. {hi:.2f} m (expected {rlo}-{rhi})')
        if float(valid.std()) > 1.0:
            r.ok(group, 'depth not constant', f'std = {valid.std():.1f} mm')
        else:
            r.fail(group, 'depth not constant',
                   'std ~ 0, the sensor may be covered or frozen')


def check_tf(node, r):
    group = 'TF'
    for parent, child in TF_CHAIN:
        label = f'{parent} -> {child}'
        try:
            tf = node.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001 - tf2 raises several types
            message = str(exc).split('\n')[0]
            r.fail(group, label, message[:70])
            continue

        t = tf.transform.translation
        q = tf.transform.rotation
        values = [t.x, t.y, t.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in values):
            r.fail(group, label, 'contains NaN')
            continue
        norm = math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
        if abs(norm - 1.0) > 1e-3:
            r.fail(group, label, f'rotation not normalised (|q|={norm:.4f})')
            continue
        r.ok(group, label,
             f'xyz = [{t.x:+.3f} {t.y:+.3f} {t.z:+.3f}]')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=6.0,
                        help='seconds to sample topics (default 6)')
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = SensorCheck(args.duration)

    print(f'sampling for {args.duration:.0f}s ... '
          f'(keep the robot still for the IMU checks)')
    node.collect()

    r = Result()
    check_rates(node, r)
    check_imu(node, r)
    check_joints(node, r)
    check_camera(node, r)
    check_tf(node, r)
    failed = r.render()

    print(f'\n{DIM}Not checkable from a script -- do these in RViz2:{RESET}')
    print('  1. Optical frame  : show the depth cloud with fixed frame '
          'base_link, point at a flat floor. It must lie flat and BELOW the '
          'robot, not rotated 90 deg.')
    print('  2. Extrinsic      : put a box at a measured distance/height. Its '
          'cloud position in base_link must match the tape measure.')
    print('  3. Gravity        : standing still, the floor plane in base_link '
          'must be level.')
    print('  4. IMU axes       : tilt forward -> pitch increases; roll left -> '
          'roll increases (REP-103).')

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())

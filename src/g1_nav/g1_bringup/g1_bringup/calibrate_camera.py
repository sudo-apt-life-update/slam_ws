#!/usr/bin/env python3
"""Measure the camera bracket angle. Run this at the start of every day.

WHY DAILY

The D435i sits on a bracket that is adjusted by hand, and `d435_joint` in the
URDF is a CALIBRATED value, not a nominal one. The moment the bracket moves,
the URDF describes a camera that no longer exists, and every point cloud is
silently tilted by the difference. That has already happened twice:

    2026-08-08   63.9 deg -> 19.16 deg   (deliberate re-mount, recalibrated)
    2026-08-20   found at 35.87 deg      (moved between sessions, NOT noticed
                                          until the scan turned out to be
                                          measuring floor instead of walls)

Nothing warns you. The stack keeps running, the checks keep passing, and the
map quietly comes out wrong. So: measure first, then work.

HOW IT MEASURES

Two independent sensors have to agree about which way is down.

    camera : fit the floor plane in the depth image (RANSAC on the lower rows).
             With the standard optical rotation, a camera pitched theta below
             horizontal sees world-up at [0, -cos(theta), -sin(theta)], so
             theta = atan2(-u_z, -u_y).
    IMU    : the accelerometer at rest gives the robot's own lean, which is
             subtracted to leave the angle relative to the torso -- which is
             what d435_joint actually encodes.

Deliberately NOT derived through the URDF chain: that would fold in the very
value being measured, and a sign error there is invisible.

Usage:
    ros2 run g1_bringup calibrate_camera              # measure and compare
    ros2 run g1_bringup calibrate_camera --write      # also update the URDF
"""

import argparse
import json
import math
import os
import sys
import time
from datetime import datetime

import numpy as np
import rclpy
import rclpy.node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu

HISTORY = os.path.expanduser('~/.ros/g1_checks/camera_calibration.json')
URDF = os.path.expanduser(
    '~/workspaces/slam_ws/src/g1_nav/g1_description/urdf/g1_29dof.urdf')

# Camera height above the floor, from base_footprint -> camera_color_optical.
CAMERA_HEIGHT = 1.2589
# D435 vertical field of view at 848x480 (fy = 607.3).
VERTICAL_FOV = 43.1
# The scan band the AMCL pipeline filters to (g1_perception/config/scan.yaml).
BAND_LOW, BAND_HIGH = 0.30, 1.50

# ACCEPTABLE RANGE FOR AMCL, derived rather than guessed.
#
# AMCL localises off walls, so what matters is how much of the 0.3-1.5 m scan
# band is still visible at the ranges it matches over (say 4 m). The top edge
# of the frame sits at (VERTICAL_FOV/2 - pitch) above horizontal, so the
# highest wall point visible at distance d is
#
#     1.2589 - d * tan(pitch - VERTICAL_FOV/2)
#
# At 4 m that gives:
#     15 deg -> whole band visible
#     20 deg -> 1.37 m, ~91% of the band
#     25 deg -> 1.02 m, ~55%
#     30 deg -> 0.67 m, ~27%
#     35 deg -> 0.30 m, nothing usable
#
# Below about 12 deg the camera is looking at the ceiling and loses any view of
# the ground near the feet, which costs obstacle awareness for no localisation
# gain.
AMCL_IDEAL = (15.0, 25.0)
AMCL_LIMIT = (12.0, 30.0)

GREEN, RED, YELLOW, DIM, BOLD, RESET = (
    '\033[32m', '\033[31m', '\033[33m', '\033[2m', '\033[1m', '\033[0m')


def visible_wall_top(pitch_deg, distance):
    """Highest wall point visible at `distance`, metres above the floor."""
    edge = math.radians(pitch_deg - VERTICAL_FOV / 2.0)
    return CAMERA_HEIGHT - distance * math.tan(edge)


def nearest_visible_floor(pitch_deg):
    """Horizontal distance to the closest floor the camera can see."""
    lower = math.radians(pitch_deg + VERTICAL_FOV / 2.0)
    return CAMERA_HEIGHT / math.tan(lower) if lower < math.pi / 2 else 0.0


def band_fraction(pitch_deg, distance=4.0):
    """How much of the scan band is visible at `distance`, as a fraction."""
    top = min(visible_wall_top(pitch_deg, distance), BAND_HIGH)
    return max(0.0, (top - BAND_LOW)) / (BAND_HIGH - BAND_LOW)


def fit_plane(points, tolerance, iterations=250, seed=0):
    rng = np.random.default_rng(seed)
    best_normal, best_offset, best_count = None, None, 0
    for _ in range(iterations):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1e-9:
            continue
        normal = normal / length
        offset = normal @ a
        count = int((np.abs(points @ normal - offset) < tolerance).sum())
        if count > best_count:
            best_normal, best_offset, best_count = normal, offset, count
    if best_normal is None:
        return None, None, 0.0
    mask = np.abs(points @ best_normal - best_offset) < tolerance
    inliers = points[mask]
    centroid = inliers.mean(axis=0)
    _, _, vt = np.linalg.svd(inliers - centroid, full_matrices=False)
    normal = vt[2] / np.linalg.norm(vt[2])
    residual = float(np.std(inliers @ best_normal - best_offset))
    return normal, residual, float(mask.mean())


class Calibrator(rclpy.node.Node):
    def __init__(self):
        super().__init__('calibrate_camera')
        reliable = QoSProfile(depth=5,
                              reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        best_effort = QoSProfile(depth=50,
                                 reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)
        self.depth = None
        self.info = None
        self.imu = []
        self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw',
            lambda m: setattr(self, 'depth', m), reliable)
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            lambda m: setattr(self, 'info', m), reliable)
        self.create_subscription(
            Imu, '/imu/data', lambda m: self.imu.append(m), best_effort)

    def ready(self, seconds=20.0):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.depth is not None and self.info is not None \
                    and len(self.imu) > 40:
                return True
        return False

    def sample(self, seed):
        """One (pitch, inlier fraction, residual, tolerance) or None."""
        message = self.depth
        depth = np.frombuffer(
            message.data, dtype=np.uint16).reshape(
                message.height, message.width).astype(np.float32) / 1000.0
        k = self.info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]

        row0 = int(depth.shape[0] * 0.70)
        rows, cols = np.mgrid[row0:depth.shape[0], 0:depth.shape[1]]
        z = depth[row0:, :]
        valid = (z > 0.3) & (z < 6.0)
        if valid.sum() < 2000:
            return None
        z = z[valid]
        points = np.column_stack([(cols[valid] - cx) * z / fx,
                                  (rows[valid] - cy) * z / fy, z])
        if len(points) > 20000:
            points = points[::len(points) // 20000]

        # Adaptive tolerance. A matte floor fits inside 2 cm; the glossy epoxy
        # floor in the machine hall scatters IR and needs 5 cm. Starting tight
        # and loosening keeps the good case honest instead of always accepting
        # a sloppy fit.
        for tolerance in (0.02, 0.035, 0.05):
            normal, residual, inliers = fit_plane(points, tolerance, seed=seed)
            if normal is not None and inliers >= 0.60:
                break
        else:
            return None

        if normal[1] > 0:                 # optical +y is down
            normal = -normal
        pitch = math.degrees(math.atan2(-normal[2], -normal[1]))

        accel = np.array([[s.linear_acceleration.x, s.linear_acceleration.y,
                           s.linear_acceleration.z] for s in self.imu[-40:]])
        gravity = accel.mean(axis=0)
        magnitude = float(np.linalg.norm(gravity))
        gravity = gravity / magnitude
        lean = math.degrees(math.atan2(gravity[0], gravity[2]))

        # Subtract the robot's own lean to get the angle relative to the torso,
        # which is what d435_joint encodes.
        return pitch + lean, inliers, residual, tolerance, magnitude


def load_history():
    if not os.path.exists(HISTORY):
        return []
    try:
        with open(HISTORY) as handle:
            return json.load(handle)
    except Exception:                     # noqa: BLE001
        return []


def save_history(entries):
    os.makedirs(os.path.dirname(HISTORY), exist_ok=True)
    with open(HISTORY, 'w') as handle:
        json.dump(entries, handle, indent=2)


def verdict(pitch):
    if AMCL_IDEAL[0] <= pitch <= AMCL_IDEAL[1]:
        return 'ideal', GREEN
    if AMCL_LIMIT[0] <= pitch <= AMCL_LIMIT[1]:
        return 'usable', YELLOW
    return 'out of range', RED


def report(pitch, spread, count, quality, history):
    previous = history[-1] if history else None

    print(f'\n{BOLD}CAMERA BRACKET ANGLE{RESET}')
    print(f'{"":14s} {"date":<12s} {"pitch":>16s} {"floor from":>11s} '
          f'{"wall @4m":>9s} {"band":>6s}')

    def row(label, entry_pitch, date, spread_text=''):
        print(f'  {label:<12s} {date:<12s} '
              f'{entry_pitch:>8.2f} deg{spread_text:<7s} '
              f'{nearest_visible_floor(entry_pitch):>9.2f} m '
              f'{visible_wall_top(entry_pitch, 4.0):>7.2f} m '
              f'{100 * band_fraction(entry_pitch):>5.0f}%')

    if previous:
        row('previous', previous['pitch_deg'], previous['date'])
    else:
        print(f'  {DIM}previous     (no earlier calibration recorded){RESET}')
    row('TODAY', pitch, datetime.now().strftime('%Y-%m-%d'),
        f' +/-{spread:.2f}')

    if previous:
        change = pitch - previous['pitch_deg']
        days = previous['date']
        if abs(change) < 1.0:
            print(f'\n  change since {days}: {GREEN}{change:+.2f} deg{RESET} '
                  f'-- bracket has not moved')
        else:
            print(f'\n  change since {days}: {YELLOW}{change:+.2f} deg{RESET} '
                  f'-- the bracket HAS moved')

    label, colour = verdict(pitch)
    print(f'\n  AMCL suitability: {colour}{label}{RESET}   '
          f'(ideal {AMCL_IDEAL[0]:.0f}-{AMCL_IDEAL[1]:.0f} deg, '
          f'usable {AMCL_LIMIT[0]:.0f}-{AMCL_LIMIT[1]:.0f})')
    if label == 'out of range':
        if pitch > AMCL_LIMIT[1]:
            print(f'  {RED}Too steep.{RESET} Only '
                  f'{100 * band_fraction(pitch):.0f}% of the scan band is '
                  f'visible at 4 m -- AMCL will have little wall to match. '
                  f'Raise the bracket.')
        else:
            print(f'  {RED}Too shallow.{RESET} The camera cannot see the '
                  f'ground within {nearest_visible_floor(pitch):.1f} m, '
                  f'losing near-field obstacle awareness. Lower the bracket.')

    print(f'\n{DIM}  measured from {count} samples, {quality}{RESET}')
    print(f'{DIM}  URDF d435_joint rpy y = {math.radians(pitch):.7f} rad'
          f'{RESET}')


def update_urdf(pitch):
    """Rewrite d435_joint's rpy. Returns (ok, message)."""
    import re
    if not os.path.exists(URDF):
        return False, f'URDF not found at {URDF}'
    source = open(URDF).read()
    pattern = re.compile(
        r'(<joint name="d435_joint" type="fixed">\s*<origin xyz="[^"]*" '
        r'rpy="0 )([-0-9.]+)( 0"/>)')
    match = pattern.search(source)
    if not match:
        return False, 'could not find d435_joint rpy in the URDF'
    old = float(match.group(2))
    new = math.radians(pitch)
    source = pattern.sub(lambda m: m.group(1) + f'{new:.7f}' + m.group(3),
                         source, count=1)
    open(URDF, 'w').write(source)
    return True, (f'{old:.7f} -> {new:.7f} rad '
                  f'({math.degrees(old):.2f} -> {pitch:.2f} deg)')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=int, default=12)
    parser.add_argument('--write', action='store_true',
                        help='update d435_joint in the URDF. Rebuild '
                             'g1_description afterwards.')
    parser.add_argument('--no-record', action='store_true',
                        help='do not add this run to the history')
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Calibrator()

    if not node.ready():
        print('no camera + IMU. Is navigation.launch.py running, and the '
              'camera server up on the robot?')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1

    print(f'{DIM}measuring -- keep the robot still, with open floor in '
          f'view{RESET}')
    samples, inliers, residuals, tolerances, gravities = [], [], [], [], []
    for i in range(args.samples):
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.02)
        result = node.sample(i)
        if result is None:
            print(f'  sample {i + 1:2d}: {YELLOW}no usable floor plane{RESET}')
            continue
        pitch, inlier, residual, tolerance, magnitude = result
        samples.append(pitch)
        inliers.append(inlier)
        residuals.append(residual)
        tolerances.append(tolerance)
        gravities.append(magnitude)
        print(f'  sample {i + 1:2d}: {pitch:6.2f} deg   '
              f'inliers {100 * inlier:3.0f}%   '
              f'residual {residual * 1000:2.0f} mm')

    if len(samples) < 5:
        print(f'\n{RED}only {len(samples)} usable samples{RESET} -- point the '
              f'camera at open floor and retry.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1

    values = np.array(samples)
    # Trim the extremes before averaging: a single bad plane fit on a
    # reflective floor can pull the mean by a degree.
    if len(values) >= 8:
        values = np.sort(values)[1:-1]
    pitch = float(values.mean())
    spread = float(values.std())

    if abs(np.mean(gravities) - 9.81) > 0.3:
        print(f'\n{YELLOW}the robot was moving ({np.mean(gravities):.2f} '
              f'm/s^2) -- the IMU lean is unreliable, so this angle is '
              f'suspect.{RESET}')

    quality = (f'inliers {100 * np.mean(inliers):.0f}%, plane residual '
               f'{np.mean(residuals) * 1000:.0f} mm, tolerance '
               f'{max(tolerances) * 100:.0f} cm')
    history = load_history()
    report(pitch, spread, len(values), quality, history)

    if args.write:
        ok, message = update_urdf(pitch)
        print(f'\n  URDF: {GREEN + message + RESET if ok else RED + message + RESET}')
        if ok:
            print(f'  {DIM}now rebuild:  colcon build --packages-select '
                  f'g1_description{RESET}')

    if not args.no_record:
        history.append({
            'date': datetime.now().strftime('%Y-%m-%d'),
            'time': datetime.now().strftime('%H:%M:%S'),
            'pitch_deg': pitch,
            'spread_deg': spread,
            'samples': len(values),
            'inlier_fraction': float(np.mean(inliers)),
            'plane_residual_m': float(np.mean(residuals)),
            'written_to_urdf': bool(args.write),
        })
        save_history(history)
        print(f'{DIM}  recorded in {HISTORY}{RESET}')

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

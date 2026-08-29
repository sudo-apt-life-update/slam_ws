#!/usr/bin/env python3
"""Verify the IMU's ROLL axis by racing it against the camera, robot stationary.

WHY ROLL, AND WHY ONLY ROLL
---------------------------
The pitch axis is already confirmed. During the camera mount calibration the
camera measured its own angle against the floor, the IMU's reported lean was
subtracted to get the angle relative to the torso, and afterwards the floor
landed 7.73 deg tilted in base_footprint while the IMU independently reported
the pelvis leaning 7.46 deg. Those only agree if the pitch sign is right.

Roll never got that treatment: the robot stayed within ~1 deg of level in roll
throughout, and a sign flip on a quantity that is essentially zero is
undetectable. So roll is the real gap, and it is what this closes.

WHY STATIONARY, NOT WALKING
---------------------------
An earlier version recorded during a walk and used the fused quaternion,
because the accelerometer only reads "up" when still. That was a mistake twice
over:

  * The two plausible readings of the quaternion (world-from-body vs
    body-from-world) differ by a SIGN FLIP on exactly the axes under test.
    Near upright they are indistinguishable -- measured 0.93 deg vs 0.61 deg,
    a coin toss -- so the test could not tell "I read the quaternion wrong"
    from "the axis is inverted".
  * The robot is tethered by ethernet. Walking yanked the cable: 11 link drops
    in 90 s, several renegotiating at 100 Mbps, which cannot even carry the
    152 Mbit/s camera stream.

The accelerometer has no such ambiguity -- at rest it measures the gravity
vector directly in the IMU frame -- and a stationary robot cannot pull its own
cable out. Hence: hold still at a few roll angles.

A useful by-product: at a 10-15 deg tilt the two quaternion candidates separate
by 20-30 deg instead of 0.3 deg, so this run also settles the quaternion
convention, which the EKF needs anyway.

WHAT IT COMPARES
----------------
Both sensors see the same physical vector -- world-up in the pelvis frame --
so they are compared directly, no angle conventions:

  IMU     : accelerometer at rest, normalised.
  Camera  : floor plane from the depth image (RANSAC on the lower rows), giving
            world-up in the optical frame, carried into pelvis through TF.

Then fit   imu_y = slope * camera_y + offset

  slope ~ +1  roll axis agrees
  slope ~ -1  roll axis is INVERTED
  slope ~  0  no response, axes permuted

Every accepted sample and all metadata are written to JSON under
~/.ros/g1_checks/ so runs can be compared after any hardware change.

Usage:
    ros2 run g1_bringup check_imu_axes [--seconds 120] [--out DIR]
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime

import numpy as np
import rclpy
import rclpy.node
from rclpy.duration import Duration
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_ros import Buffer, TransformListener

# 0.10 of a unit vector is about 6 deg. Below that a slope means nothing.
MIN_RANGE = 0.10
# "Still enough" for the accelerometer to be measuring gravity alone.
STILL_ACCEL_TOL = 0.25          # m/s^2 away from 9.81
STILL_GYRO_MAX = 0.05           # rad/s

GREEN, RED, YELLOW, DIM, RESET = (
    '\033[32m', '\033[31m', '\033[33m', '\033[2m', '\033[0m')


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def fit_plane_ransac(points, iterations=150, tolerance=0.02, seed=0):
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
        return None, 0.0
    mask = np.abs(points @ best_normal - best_offset) < tolerance
    inliers = points[mask]
    centroid = inliers.mean(axis=0)
    _, _, vt = np.linalg.svd(inliers - centroid, full_matrices=False)
    return vt[2] / np.linalg.norm(vt[2]), float(mask.mean())


class RollCheck(rclpy.node.Node):
    def __init__(self):
        super().__init__('check_imu_axes')
        self.buffer = Buffer(cache_time=Duration(seconds=20))
        TransformListener(self.buffer, self)

        reliable = QoSProfile(depth=5,
                              reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        best_effort = QoSProfile(depth=200,
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
        self.create_subscription(Imu, '/imu/data', self._on_imu, best_effort)

    def _on_imu(self, msg):
        self.imu.append(msg)
        if len(self.imu) > 2000:
            del self.imu[:1000]

    def ready(self, seconds=20.0):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.depth is not None and self.info is not None \
                    and len(self.imu) > 50:
                return True
        return False

    def imu_state(self, window=40):
        """(up from accel, |accel|, |gyro|, both quaternion candidates)."""
        recent = self.imu[-window:]
        accel = np.array([[m.linear_acceleration.x, m.linear_acceleration.y,
                           m.linear_acceleration.z] for m in recent])
        gyro = np.array([[m.angular_velocity.x, m.angular_velocity.y,
                          m.angular_velocity.z] for m in recent])
        mean = accel.mean(axis=0)
        magnitude = float(np.linalg.norm(mean))
        q = recent[-1].orientation
        matrix = quat_to_matrix(q.x, q.y, q.z, q.w)
        candidates = {'row': matrix[2, :].tolist(),
                      'col': matrix[:, 2].tolist()}
        return (mean / magnitude, magnitude,
                float(np.linalg.norm(gyro, axis=1).mean()), candidates)

    def camera_up(self):
        """(up in pelvis, normal in optical, inlier fraction) or (None, ...)."""
        if self.depth is None or self.info is None:
            return None, None, 0.0
        message = self.depth
        depth = np.frombuffer(
            message.data, dtype=np.uint16).reshape(
                message.height, message.width).astype(np.float32) / 1000.0
        k = self.info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]

        row0 = int(depth.shape[0] * 0.5)
        rows, cols = np.mgrid[row0:depth.shape[0], 0:depth.shape[1]]
        z = depth[row0:, :]
        valid = (z > 0.3) & (z < 6.0)
        if valid.sum() < 2000:
            return None, None, 0.0
        z = z[valid]
        points = np.column_stack([(cols[valid] - cx) * z / fx,
                                  (rows[valid] - cy) * z / fy, z])
        if len(points) > 15000:
            points = points[::len(points) // 15000]

        normal, inlier_fraction = fit_plane_ransac(points)
        if normal is None or inlier_fraction < 0.45:
            return None, None, inlier_fraction
        if normal[1] > 0:                     # optical +y is down
            normal = -normal

        try:
            tf = self.buffer.lookup_transform(
                'pelvis', message.header.frame_id, rclpy.time.Time())
        except Exception:                     # noqa: BLE001
            return None, None, inlier_fraction
        r = tf.transform.rotation
        up = quat_to_matrix(r.x, r.y, r.z, r.w) @ normal
        return up / np.linalg.norm(up), normal, inlier_fraction


def component_noise(values, window=20):
    """Scatter of a component while the pose is essentially constant.

    Used to tell "the robot never tilted" from "the axis does not respond".
    Both give a slope near zero, but only the latter has real motion on the
    camera side, so the span has to be judged against this noise floor rather
    than a fixed number.
    """
    if len(values) < 2 * window:
        return float(np.std(values))
    spreads = [np.std(values[i:i + window])
               for i in range(0, len(values) - window, window)]
    return float(np.median(spreads))


def orthogonal_slope(x, y):
    """Total-least-squares slope: allows for noise in x as well as y.

    Ordinary regression is biased toward zero when the INPUT is noisy
    (attenuation), and the camera's floor-normal estimate is noisy. On the
    first real run that bias alone dragged the roll slope from ~0.96 to 0.87,
    which read as a failure when the data was fine.
    """
    xm, ym = x.mean(), y.mean()
    _, _, vt = np.linalg.svd(np.column_stack([x - xm, y - ym]),
                             full_matrices=False)
    direction = vt[0]
    if abs(direction[0]) < 1e-12:
        return float('nan')
    return float(direction[1] / direction[0])


def slope_stderr(x, y):
    """1-sigma standard error on the ordinary slope."""
    n = len(x)
    design = np.column_stack([x, np.ones(n)])
    coefficients, *_ = np.linalg.lstsq(design, y, rcond=None)
    residual = y - design @ coefficients
    if n <= 2:
        return float('inf')
    variance = (residual ** 2).sum() / (n - 2)
    covariance = variance * np.linalg.inv(design.T @ design)
    return float(np.sqrt(np.diag(covariance))[0])


def best_fit_rotation(source, target):
    """Kabsch: the rotation carrying `source` onto `target`.

    The honest way to compare two sets of direction vectors. Per-axis
    regression silently assumes the axes moved independently, but a hand tilt
    moves pitch and roll together, so the components are cross-coupled.
    An inverted axis shows up here as ~180 deg; a mounting offset as a few deg.
    """
    h = source.T @ target
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    rotation = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    angle = float(np.degrees(np.arccos(
        np.clip((np.trace(rotation) - 1) / 2, -1, 1))))
    axis = np.array([rotation[2, 1] - rotation[1, 2],
                     rotation[0, 2] - rotation[2, 0],
                     rotation[1, 0] - rotation[0, 1]])
    norm = np.linalg.norm(axis)
    axis = axis / norm if norm > 1e-9 else np.array([0.0, 0.0, 1.0])
    residual = target - (rotation @ source.T).T
    return rotation, angle, axis, float(
        np.degrees(np.linalg.norm(residual, axis=1)).mean())


def analyse(samples):
    """Decide each axis from the slope and its confidence interval."""
    imu = np.array([s['imu_up'] for s in samples])
    cam = np.array([s['cam_up'] for s in samples])
    result = {'n_samples': len(samples), 'axes': {}}
    failures = 0

    print(f'\n{DIM}{len(samples)} stationary samples{RESET}\n')
    print(f'{"axis":6} {"span":>7} {"OLS":>7} {"TLS":>7} {"95% interval":>18}'
          f'   verdict')

    for index, (name, meaning) in enumerate([('x', 'pitch'), ('y', 'roll')]):
        c, i = cam[:, index], imu[:, index]
        span = float(c.max() - c.min())
        ols = float(np.polyfit(c, i, 1)[0])
        tls = orthogonal_slope(c, i)
        stderr = slope_stderr(c, i)
        low, high = ols - 1.96 * stderr, ols + 1.96 * stderr
        noise = component_noise(c)
        # The camera has to have actually moved by more than its own scatter,
        # or a near-zero slope means "you did not tilt it", not "the axis is
        # dead". Six sigma of the noise floor is the bar.
        moved = span > max(0.01, 6.0 * noise)
        entry = {'range': span, 'slope_ols': ols, 'slope_tls': tls,
                 'stderr': stderr, 'ci95': [low, high],
                 'camera_noise': noise, 'moved': bool(moved)}

        # The question is "is this axis inverted or dead", not "is the slope
        # exactly 1". Decide on what the interval excludes, so a small but
        # well-measured span still gives an answer.
        if not np.isfinite(stderr) or not moved or stderr > 0.3:
            note = (f'{YELLOW}INCONCLUSIVE{RESET} - tilt further '
                    f'(span {span:.3f} vs noise {noise:.3f})')
            entry['verdict'] = 'inconclusive'
            failures += 1
        elif low > 0.5:
            note = f'{GREEN}OK{RESET} - {meaning} axis agrees'
            entry['verdict'] = 'ok'
        elif high < -0.5:
            note = f'{RED}INVERTED{RESET} - negate the {meaning} axis'
            entry['verdict'] = 'inverted'
            failures += 1
        elif low > -0.3 and high < 0.3:
            note = f'{RED}NO RESPONSE{RESET} - axes look permuted'
            entry['verdict'] = 'no_response'
            failures += 1
        else:
            note = f'{YELLOW}UNCLEAR{RESET} - interval spans too much'
            entry['verdict'] = 'unclear'
            failures += 1

        print(f'{name:6} {span:7.3f} {ols:+7.3f} {tls:+7.3f} '
              f'[{low:+.3f},{high:+.3f}]   {note}')
        result['axes'][meaning] = entry

    angle = np.degrees(np.arccos(np.clip((imu * cam).sum(axis=1), -1, 1)))
    result['up_vector_disagreement_deg'] = {
        'mean': float(angle.mean()), 'max': float(angle.max())}
    print(f'\nangle between the two up-vectors: mean {angle.mean():.2f} deg, '
          f'max {angle.max():.2f} deg')

    _, rot_angle, rot_axis, rot_residual = best_fit_rotation(cam, imu)
    result['best_fit_rotation'] = {
        'angle_deg': rot_angle, 'axis': rot_axis.tolist(),
        'residual_deg': rot_residual}
    print(f'{DIM}best-fit rotation camera-up -> imu-up: {rot_angle:.2f} deg '
          f'about [{rot_axis[0]:+.2f} {rot_axis[1]:+.2f} {rot_axis[2]:+.2f}], '
          f'residual {rot_residual:.2f} deg{RESET}')
    # Rotation about z leaves an up-vector unchanged, so only the in-plane part
    # is identifiable. Report that, not the raw magnitude.
    in_plane = rot_angle * float(np.linalg.norm(rot_axis[:2]))
    result['best_fit_rotation']['in_plane_deg'] = in_plane
    if in_plane > 2.0:
        print(f'  {YELLOW}{in_plane:.1f} deg of fixed misalignment{RESET} '
              f'(pitch {rot_angle * rot_axis[1]:+.1f}, '
              f'roll {rot_angle * -rot_axis[0]:+.1f}). Not an axis error, but '
              f'it will tilt the map; the floor may also not be level.')

    # Quaternion convention: only separable once genuinely tilted.
    tilt = np.degrees(np.arccos(np.clip(imu[:, 2], -1, 1)))
    tilted = tilt > 5.0
    print(f'\n{DIM}quaternion convention ({int(tilted.sum())} samples tilted '
          f'>5 deg){RESET}')
    if tilted.sum() < 5:
        print(f'  {YELLOW}too few tilted samples to decide{RESET} - near '
              f'upright the two readings are indistinguishable')
        result['quaternion_convention'] = {'decided': False}
    else:
        errors = {}
        for name in ('row', 'col'):
            vectors = np.array([s['quat_' + name] for s in samples])[tilted]
            vectors = vectors / np.linalg.norm(vectors, axis=1, keepdims=True)
            reference = imu[tilted]
            errors[name] = float(np.degrees(np.arccos(np.clip(
                (vectors * reference).sum(axis=1), -1, 1))).mean())
        best = min(errors, key=errors.get)
        margin = abs(errors['row'] - errors['col'])
        for name in ('row', 'col'):
            print(f'  as {name}: {errors[name]:6.2f} deg from gravity')
        decided = margin > 5.0 and errors[best] < 5.0
        result['quaternion_convention'] = {
            'decided': bool(decided), 'best': best,
            'errors_deg': errors, 'margin_deg': margin}
        if decided:
            print(f'  -> {GREEN}{best}{RESET} (margin {margin:.1f} deg)')
        else:
            print(f'  -> {YELLOW}not conclusive{RESET} (margin {margin:.1f} '
                  f'deg); tilt further and re-run')

    print()
    roll = result['axes'].get('roll', {}).get('verdict')
    if roll == 'ok':
        print(f'{GREEN}PASS{RESET} - the roll axis agrees with the camera and '
              f'the world.')
    else:
        print(f'{RED}NOT VERIFIED{RESET} - see above. Do not start the EKF '
              f'until the roll axis is clean.')
    result['failures'] = failures
    return result, failures


def write_json(path, payload):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as handle:
        json.dump(payload, handle, indent=2)
    print(f'{DIM}recorded {payload["result"]["n_samples"]} samples -> '
          f'{path}{RESET}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=120.0)
    parser.add_argument('--out', default=os.path.expanduser('~/.ros/g1_checks'))
    parser.add_argument(
        '--reanalyse', metavar='JSON',
        help='re-run the analysis on a saved run instead of the robot. The '
             'raw samples are kept, so an improved analysis can be applied '
             'to old data without touching the hardware.')
    args, ros_args = parser.parse_known_args(argv)

    if args.reanalyse:
        with open(args.reanalyse) as handle:
            saved = json.load(handle)
        print(f'{DIM}re-analysing {args.reanalyse} '
              f'({len(saved["samples"])} samples){RESET}')
        _, failures = analyse(saved['samples'])
        return 1 if failures else 0

    rclpy.init(args=ros_args)
    node = RollCheck()

    if not node.ready():
        print('no camera + IMU data. Is sensors.launch.py running?')
        rclpy.shutdown()
        return 1

    print(f'{DIM}ROLL the robot SIDEWAYS and hold still at each angle.{RESET}')
    print('   left side down ~10-15 deg, HOLD 3 s')
    print('   upright, HOLD 3 s')
    print('   right side down ~10-15 deg, HOLD 3 s')
    print('   repeat 3-4 times, with a few part-way angles')
    print(f'{DIM}Samples are only taken while still -- the accelerometer '
          f'cannot measure gravity while you are moving it.{RESET}\n')

    samples = []
    rejected = {'moving': 0, 'nofit': 0}
    end = time.time() + args.seconds
    last_print = 0.0
    interrupted = False

    try:
        while time.time() < end and rclpy.ok():
            for _ in range(8):
                rclpy.spin_once(node, timeout_sec=0.02)

            imu_up, magnitude, gyro, candidates = node.imu_state()
            still = (abs(magnitude - 9.81) < STILL_ACCEL_TOL
                     and gyro < STILL_GYRO_MAX)
            cam_up, normal, inliers = node.camera_up()

            if cam_up is None:
                rejected['nofit'] += 1
            elif not still:
                rejected['moving'] += 1
            else:
                samples.append({
                    'time': time.time(),
                    'imu_up': imu_up.tolist(),
                    'accel_magnitude': magnitude,
                    'gyro_magnitude': gyro,
                    'quat_row': candidates['row'],
                    'quat_col': candidates['col'],
                    'cam_up': cam_up.tolist(),
                    'floor_normal_optical': normal.tolist(),
                    'plane_inlier_fraction': inliers,
                })

            if time.time() - last_print > 0.4:
                last_print = time.time()
                if samples:
                    c = np.array([s['cam_up'] for s in samples])
                    span = c[:, 1].max() - c[:, 1].min()
                    bar = (f'roll range {span:.3f}'
                           + (f' {GREEN}(enough){RESET}' if span > MIN_RANGE
                              else f' {DIM}(need {MIN_RANGE}){RESET}'))
                else:
                    bar = 'waiting for a floor fit'
                state = (f'{GREEN}still{RESET}' if still
                         else f'{YELLOW}moving{RESET}')
                print(f'\r  {len(samples):4d} samples  {state}  {bar}   '
                      f'(dropped {rejected["nofit"]} nofit / '
                      f'{rejected["moving"]} moving)     ', end='', flush=True)
    except KeyboardInterrupt:
        interrupted = True
        print(f'\n{YELLOW}interrupted{RESET} - analysing what was collected')

    print()
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(args.out, f'imu_roll_{stamp}.json')

    if len(samples) < 15:
        print(f'{RED}only {len(samples)} samples{RESET} - not enough. Keep the '
              f'floor in view and hold still at each angle.')
        write_json(path, {'metadata': {'stamp': stamp,
                                       'interrupted': interrupted,
                                       'rejected': rejected},
                          'result': {'n_samples': len(samples),
                                     'verdict': 'insufficient'},
                          'samples': samples})
        node.destroy_node()
        rclpy.shutdown()
        return 1

    result, failures = analyse(samples)

    info = node.info
    write_json(path, {
        'metadata': {
            'stamp': stamp,
            'interrupted': interrupted,
            'rejected': rejected,
            'camera': {'width': info.width, 'height': info.height,
                       'k': list(info.k)},
            'still_criteria': {'accel_tol': STILL_ACCEL_TOL,
                               'gyro_max': STILL_GYRO_MAX},
        },
        'result': result,
        'samples': samples,
    })

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

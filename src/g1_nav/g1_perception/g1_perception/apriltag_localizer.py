#!/usr/bin/env python3
"""Detect an AprilTag and use it as a fixed landmark to kill accumulated drift.

WHY THIS EXISTS

Scan matching alone is losing the fight on this robot. A 70 degree FOV whose
returns start at ~1.5 m gives weak geometric constraints, so pose error grows
faster than loop closure can remove it. Three walks in:

    walk 2   ribbon, never closed, AMCL failed (62% of scans landed in unknown)
    walk 3   full perimeter walked, returned to the start -- and the map is
             still an open horseshoe. slam_toolbox never even SEARCHED for the
             loop, because the estimate had drifted past
             loop_search_maximum_distance (3.0 m) and it no longer believed it
             was anywhere near the old nodes.

A fiducial cuts through that. The tag does not drift. Seeing it re-establishes
an absolute position regardless of how far the estimate has wandered, which is
exactly the thing scan matching cannot do here.

WHAT IT PUBLISHES

    /tag_detections      geometry_msgs/PoseStamped   tag pose in the camera
                                                     optical frame
    /initialpose         PoseWithCovarianceStamped   (only with
                                                     publish_initialpose:=true)
    TF  <camera_optical> -> tag_<id>                 for visual confirmation

HOW IT IS USED

Two modes, and the difference matters:

  MAPPING    Run it to SEE the tag. When the tag reappears at the end of a
             circuit, compare its reported map position against where it was
             first seen -- the difference IS the accumulated drift, measured
             rather than guessed.

  LOCALISING With a known tag pose in the map (tag_map_x/y/yaw), seeing the tag
             gives an absolute robot pose, published to /initialpose. AMCL
             snaps to it. This is a re-localisation aid, not a replacement for
             scan matching: it only fires when the tag is actually in view.

DETECTION

The tag is AprilTag 36h11, id 10, 16 cm. OpenCV's aruco module reads that
family directly (DICT_APRILTAG_36h11) -- no apriltag package needed. Pose comes
from solvePnP on the four corners with the known size, so it is full 6-DoF and
does not depend on the depth image at all. Depth is used only as a sanity
cross-check, because a PnP solution can flip when the tag is viewed head-on.
"""

import math
import time

import cv2
import numpy as np
import rclpy
import rclpy.node
from cv_bridge import CvBridge
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped,
                               TransformStamped)
from rclpy.duration import Duration
from rclpy.qos import (HistoryPolicy, QoSProfile, QoSPresetProfiles,
                       ReliabilityPolicy)
from rclpy.time import Time
from rtabmap_msgs.msg import LandmarkDetection
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import (Buffer, TransformBroadcaster, TransformException,
                     TransformListener)


def quaternion_from_matrix(R):
    """Rotation matrix -> (x, y, z, w)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        return ((R[2, 1] - R[1, 2]) * s, (R[0, 2] - R[2, 0]) * s,
                (R[1, 0] - R[0, 1]) * s, 0.25 / s)
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s)
    if R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s)
    s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s,
            (R[1, 0] - R[0, 1]) / s)


def matrix_from_quaternion(x, y, z, w):
    """(x, y, z, w) -> 3x3 rotation matrix. Inverse of quaternion_from_matrix."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def homogeneous(rotation, translation):
    """3x3 rotation + 3-vector translation -> 4x4."""
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


def rotation_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class AprilTagLocalizer(rclpy.node.Node):
    def __init__(self):
        super().__init__('apriltag_localizer')

        self.declare_parameter('tag_size', 0.16)          # metres, measured
        self.declare_parameter('tag_id', 10)
        self.declare_parameter('family', 'DICT_APRILTAG_36h11')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_stabilized')
        # Where the tag sits in the map. Fill these in once it is known, then
        # seeing the tag yields an absolute robot pose.
        self.declare_parameter('tag_map_x', 0.0)
        self.declare_parameter('tag_map_y', 0.0)
        self.declare_parameter('tag_map_yaw', 0.0)
        # Off by default: publishing /initialpose TELEPORTS AMCL. Useful for
        # deliberate re-localisation, disruptive if it fires on a spurious
        # detection mid-run.
        self.declare_parameter('publish_initialpose', False)
        # Minimum seconds between republishes. At 15 Hz, an unthrottled
        # republish would reset AMCL's particle filter on every frame the tag
        # is visible -- each reset discards whatever the motion model and scan
        # matching have refined since the last one. This is a re-localisation
        # AID, not a permanent external pose source: seed once, then let AMCL
        # own the estimate until the tag is needed again (e.g. after a
        # suspected kidnap).
        self.declare_parameter('initialpose_cooldown', 5.0)
        # A tag seen edge-on or far away gives a poor PnP solution. Reject
        # rather than publish a confident-looking wrong pose.
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('min_side_px', 30.0)

        # FEED THE TAG TO RTAB-MAP AS A GRAPH LANDMARK.
        #
        # This is the thing slam_toolbox could not do. RTAB-Map subscribes to
        # rtabmap_msgs/LandmarkDetection on `landmark_detection` and, with
        # Optimizer/LandmarksIgnored=false (the default), GTSAM treats each
        # sighting as a real constraint in the pose graph. Two sightings of the
        # same tag from two visits tie those nodes rigidly together -- which is
        # exactly the loop closure that never fired in four slam_toolbox walks.
        #
        # RTAB-Map wants the pose IN THE CAMERA FRAME and does the
        # base -> camera lookup itself, so what we already compute is what it
        # wants. Nothing is transformed here.
        self.declare_parameter('publish_landmark', True)

        # Uncertainty handed to the optimiser. These are deliberately LOOSER
        # than the measured repeatability (X +/-0.4 mm, Y +/-0.6 mm, Z +/-1.7 mm
        # at close range), because that figure is precision, not accuracy: it
        # says the detector returns the same answer twice, not that the answer
        # is right. Calibration error, the 3.4 deg camera-to-IMU misalignment
        # and TF timing all sit outside it.
        #
        # Linear sigma grows with range as (base + slope * distance), because a
        # 16 cm tag subtends fewer pixels the further away it is and corner
        # precision sets PnP precision.
        self.declare_parameter('landmark_sigma_base', 0.02)     # m
        self.declare_parameter('landmark_sigma_slope', 0.02)    # m per m

        # Angular sigma is FIXED AND LARGE on purpose. Orientation is the weak
        # half of a planar PnP solution -- it is the same near-degeneracy that
        # produces the flipped solutions the depth cross-check below rejects.
        # This tells GTSAM to lean on WHERE the tag is and largely disregard
        # which way it is facing.
        #
        # MEASURED 2026-08-22, walk 5. The first value tried was 0.10 rad
        # (5.7 deg), and it was too tight to be useful: the tag's measured
        # orientation disagreed with the optimised graph by 17-27 deg, so every
        # sighting looked like a 3-5 sigma contradiction and RGBD/OptimizeMaxError
        # vetoed the whole constraint -- position included:
        #
        #   Loop closure 3222->-10 rejected!
        #     abs error=27.300287 deg, stddev=0.100000 -> error ratio 4.76
        #
        # All four landmark constraints were thrown away that way, so the tag
        # contributed nothing to the map it was added to help. 0.5 rad (28.6 deg)
        # makes a 27 deg disagreement a ~1 sigma event, so the position
        # constraint survives and only the orientation is discounted -- which
        # was the intent all along.
        #
        # STILL UNRESOLVED: whether that 17-27 deg is poor PnP orientation or
        # genuine accumulated yaw drift. If it is real drift, the tag was right
        # and rejecting it discarded a correction we needed. Loosening helps in
        # either case, but do not read this value as evidence the tag's
        # orientation is untrustworthy -- that has not been established.
        self.declare_parameter('landmark_sigma_angular', 0.5)  # rad

        self.tag_size = float(self.get_parameter('tag_size').value)
        self.tag_id = int(self.get_parameter('tag_id').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_initial = bool(
            self.get_parameter('publish_initialpose').value)
        self.tag_map_x = float(self.get_parameter('tag_map_x').value)
        self.tag_map_y = float(self.get_parameter('tag_map_y').value)
        self.tag_map_yaw = float(self.get_parameter('tag_map_yaw').value)
        self.initialpose_cooldown = float(
            self.get_parameter('initialpose_cooldown').value)
        self._last_initialpose = 0.0
        self.max_range = float(self.get_parameter('max_range').value)
        self.min_side = float(self.get_parameter('min_side_px').value)
        self.publish_landmark = bool(
            self.get_parameter('publish_landmark').value)
        self.sigma_base = float(self.get_parameter('landmark_sigma_base').value)
        self.sigma_slope = float(
            self.get_parameter('landmark_sigma_slope').value)
        self.sigma_ang = float(
            self.get_parameter('landmark_sigma_angular').value)

        # RTAB-Map rejects landmark ids <= 0 outright ("IDs should be > 0"),
        # and does it inside the conversion where the message is already gone --
        # so it fails per-detection at runtime rather than at startup. Catch it
        # here instead, where the operator can still do something about it.
        if self.publish_landmark and self.tag_id <= 0:
            raise ValueError(
                f'tag_id must be > 0 to be usable as an RTAB-Map landmark, '
                f'got {self.tag_id}. Set publish_landmark:=false to run the '
                f'node purely as a drift monitor.')

        family = self.get_parameter('family').value
        self.dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, family))
        self.params = cv2.aruco.DetectorParameters_create()
        # Corner refinement matters here: PnP accuracy is set by corner
        # precision, and a 16 cm tag at 3 m is only ~30 px across.
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.bridge = CvBridge()
        self.info = None
        self.depth = None

        reliable = QoSProfile(depth=5,
                              reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self._on_info, reliable)
        self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw',
            lambda m: setattr(self, 'depth', m), reliable)
        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._on_image, reliable)

        self.pose_pub = self.create_publisher(
            PoseStamped, '/tag_detections', QoSPresetProfiles.SENSOR_DATA.value)
        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # RELIABLE, not SENSOR_DATA. RTAB-Map only reads the landmark buffer
        # when it creates a node (Rtabmap/DetectionRate, 2 Hz), so a dropped
        # detection is not "the next one will do" -- it can be the difference
        # between a revisit being tied into the graph and not. The volume is a
        # handful of small messages per second, so reliability is free here.
        self.landmark_pub = self.create_publisher(
            LandmarkDetection, '/landmark_detection', 10)
        self.broadcaster = TransformBroadcaster(self)
        self.buffer = Buffer()
        TransformListener(self.buffer, self)

        self.seen = 0
        self.rejected = 0
        self.first_map_pose = None
        self.create_timer(5.0, self._report)

        role = ('feeding /landmark_detection to RTAB-Map'
                if self.publish_landmark else 'drift monitor only')
        self.get_logger().info(
            f'looking for {family} id {self.tag_id}, '
            f'{self.tag_size * 100:.0f} cm -- {role}'
            f'{" -- WILL publish /initialpose" if self.publish_initial else ""}')

    def _on_info(self, message):
        self.info = message

    def _report(self):
        if self.seen == 0:
            self.get_logger().info(
                f'tag {self.tag_id} not in view'
                + (f' ({self.rejected} detections rejected as too far or too '
                   f'small)' if self.rejected else ''))
        else:
            self.get_logger().info(
                f'tag {self.tag_id} seen in {self.seen} frames over 5 s')
        self.seen = 0
        self.rejected = 0

    def _on_image(self, message):
        if self.info is None:
            return
        frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.params)
        if ids is None:
            return
        for tag_id, corner in zip(ids.flatten(), corners):
            if int(tag_id) != self.tag_id:
                continue
            self._handle(corner, message.header.stamp)

    def _handle(self, corner, stamp):
        points = corner.reshape(4, 2)
        side = float(np.mean([np.linalg.norm(points[i] - points[(i + 1) % 4])
                              for i in range(4)]))
        if side < self.min_side:
            self.rejected += 1
            return

        k = np.array(self.info.k, dtype=np.float64).reshape(3, 3)
        dist = np.array(self.info.d, dtype=np.float64)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            [corner], self.tag_size, k, dist)
        rvec = rvecs[0][0]
        tvec = tvecs[0][0]
        distance = float(np.linalg.norm(tvec))
        if distance > self.max_range:
            self.rejected += 1
            return

        # Cross-check PnP against the depth image. They measure the same thing
        # by different means, so a disagreement means one of them is wrong --
        # usually a flipped PnP solution, which is a known failure mode when a
        # planar tag is viewed close to head-on.
        if self.depth is not None:
            depth = np.frombuffer(self.depth.data, dtype=np.uint16).reshape(
                self.depth.height, self.depth.width).astype(np.float32) / 1000.0
            u, v = int(points[:, 0].mean()), int(points[:, 1].mean())
            window = depth[max(0, v - 5):v + 6, max(0, u - 5):u + 6]
            valid = window[(window > 0.2) & (window < 10)]
            if valid.size:
                measured = float(np.median(valid))
                if abs(measured - tvec[2]) > 0.25:
                    self.get_logger().warning(
                        f'tag pose disagrees with depth '
                        f'(PnP {tvec[2]:.2f} m vs depth {measured:.2f} m) '
                        f'-- rejecting')
                    self.rejected += 1
                    return

        self.seen += 1
        R, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = quaternion_from_matrix(R)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.info.header.frame_id
        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = float(tvec[2])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pose_pub.publish(pose)

        if self.publish_landmark:
            self._publish_landmark(pose, distance)

        if self.publish_initial:
            self._publish_initialpose(R, tvec, distance, pose.header)

        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = f'tag_{self.tag_id}'
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self.broadcaster.sendTransform(transform)

        self._track_drift(pose)

    def _publish_initialpose(self, R, tvec, distance, header):
        """Turn a tag sighting into an absolute robot pose for AMCL.

        The tag's pose in the map (tag_map_x/y/yaw) is known and fixed. The
        camera just measured the tag's pose relative to itself (R, tvec). Chain
        camera -> base_stabilized (TF, static) with base_stabilized -> tag
        (just measured) to get the tag in the robot frame, then invert against
        the tag's known map pose to solve for the robot's map pose:

            T_map_base = T_map_tag * inverse(T_base_tag)

        EVERYTHING IS PROJECTED TO 2D (x, y, yaw) before this inversion, same
        as Reg/Force3DoF in the mapping config. A vertical tag viewed by a
        levelled camera should already be close to upright, so this discards
        very little real information -- and it avoids the roll/pitch noise in
        a single PnP solution corrupting the one thing we actually need,
        which is where the robot is on the floor.
        """
        now = time.monotonic()
        if now - self._last_initialpose < self.initialpose_cooldown:
            return

        try:
            camera_to_base = self.buffer.lookup_transform(
                header.frame_id, self.base_frame, Time(),
                timeout=Duration(seconds=0.2))
        except TransformException as exc:
            self.get_logger().warning(
                f'cannot publish /initialpose -- no TF '
                f'{header.frame_id} -> {self.base_frame}: {exc}')
            return
        t = camera_to_base.transform.translation
        q = camera_to_base.transform.rotation
        base_to_camera = np.linalg.inv(homogeneous(
            matrix_from_quaternion(q.x, q.y, q.z, q.w), [t.x, t.y, t.z]))

        base_to_tag = base_to_camera @ homogeneous(R, tvec.flatten())

        # Project to (x, y, yaw). atan2(R[1,0], R[0,0]) is the yaw of a
        # rotation matrix when it is (approximately) a pure Z rotation, which
        # base_to_tag should be now that base_stabilized is level.
        bx, by = base_to_tag[0, 3], base_to_tag[1, 3]
        byaw = math.atan2(base_to_tag[1, 0], base_to_tag[0, 0])

        map_to_tag = homogeneous(rotation_z(self.tag_map_yaw),
                                 [self.tag_map_x, self.tag_map_y, 0.0])
        base_to_tag_2d = homogeneous(rotation_z(byaw), [bx, by, 0.0])
        map_to_base = map_to_tag @ np.linalg.inv(base_to_tag_2d)

        map_x, map_y = map_to_base[0, 3], map_to_base[1, 3]
        map_yaw = math.atan2(map_to_base[1, 0], map_to_base[0, 0])

        message = PoseWithCovarianceStamped()
        message.header.stamp = header.stamp
        message.header.frame_id = self.map_frame
        message.pose.pose.position.x = float(map_x)
        message.pose.pose.position.y = float(map_y)
        message.pose.pose.orientation.z = math.sin(map_yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(map_yaw / 2.0)

        # Position sigma tracks the landmark's own (range-scaled); yaw sigma is
        # doubled over the landmark's, because this pose compounds the tag's
        # orientation uncertainty with the robot's own bearing to it -- two
        # weak-orientation measurements chained, not one.
        sigma_lin = self.sigma_base + self.sigma_slope * distance
        sigma_yaw = 2.0 * self.sigma_ang
        covariance = [0.0] * 36
        covariance[0] = sigma_lin ** 2
        covariance[7] = sigma_lin ** 2
        covariance[35] = sigma_yaw ** 2
        message.pose.covariance = covariance

        self.initial_pub.publish(message)
        self._last_initialpose = now
        self.get_logger().info(
            f'/initialpose <- map ({map_x:.2f}, {map_y:.2f}), '
            f'yaw {math.degrees(map_yaw):.1f} deg, from tag {self.tag_id}')

    def _publish_landmark(self, pose, distance):
        """Hand this sighting to RTAB-Map as a pose-graph constraint.

        Only reached AFTER every rejection test above -- the size gate, the
        range gate and the depth cross-check. That ordering is deliberate: a
        landmark is a HARD constraint, so a wrong one does not degrade the map
        gracefully, it folds the graph around a lie. A detection good enough to
        report is not automatically good enough to optimise against, and the
        depth cross-check in particular exists to catch the flipped PnP
        solutions that would otherwise look entirely confident.
        """
        message = LandmarkDetection()
        # Same stamp and same frame as the PoseStamped: the colour image's
        # capture time, in the colour optical frame. RTAB-Map looks up
        # base_stabilized -> camera_color_optical_frame at this stamp and then
        # corrects for odometry motion since, so the stamp has to be the
        # CAPTURE time, not now.
        message.header = pose.header
        message.id = self.tag_id
        message.size = float(self.tag_size)
        message.pose.pose = pose.pose

        # Row-major 6x6, ordered (x, y, z, roll, pitch, yaw). Only the diagonal
        # is populated -- the off-diagonal correlations of a PnP solution are
        # real but we have no calibrated estimate of them, and inventing them
        # would be worse than declaring them unknown.
        #
        # Leaving the covariance at zero is NOT equivalent: RTAB-Map reads
        # covariance[0] <= 0 as "unset" and silently substitutes
        # landmark_linear_variance (0.001, i.e. ~3 cm sigma) regardless of how
        # far away the tag actually was.
        sigma_lin = self.sigma_base + self.sigma_slope * distance
        variances = [sigma_lin ** 2] * 3 + [self.sigma_ang ** 2] * 3
        covariance = [0.0] * 36
        for index, variance in enumerate(variances):
            covariance[index * 7] = variance
        message.pose.covariance = covariance

        self.landmark_pub.publish(message)

    def _track_drift(self, pose):
        """Where does the tag appear to be in the map, and has that moved?

        The tag is nailed to a wall, so any change in its reported MAP position
        is pure accumulated drift in the robot's own estimate. This turns drift
        from something inferred off a wonky-looking map into a number.
        """
        try:
            tf = self.buffer.lookup_transform(
                self.map_frame, pose.header.frame_id, rclpy.time.Time())
        except Exception:                                  # noqa: BLE001
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        R = np.array([
            [1 - 2 * (q.y ** 2 + q.z ** 2), 2 * (q.x * q.y - q.z * q.w),
             2 * (q.x * q.z + q.y * q.w)],
            [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x ** 2 + q.z ** 2),
             2 * (q.y * q.z - q.x * q.w)],
            [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w),
             1 - 2 * (q.x ** 2 + q.y ** 2)]])
        local = np.array([pose.pose.position.x, pose.pose.position.y,
                          pose.pose.position.z])
        world = R @ local + np.array([t.x, t.y, t.z])

        if self.first_map_pose is None:
            self.first_map_pose = world
            self.get_logger().info(
                f'tag {self.tag_id} first seen at map '
                f'({world[0]:+.2f}, {world[1]:+.2f}) -- this is the reference')
            return

        drift = float(np.linalg.norm(world[:2] - self.first_map_pose[:2]))
        if drift > 0.5:
            self.get_logger().warning(
                f'tag now appears at map ({world[0]:+.2f}, {world[1]:+.2f}), '
                f'{drift:.2f} m from where it was first seen -- that is '
                f'accumulated drift')


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

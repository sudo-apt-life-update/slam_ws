#!/usr/bin/env python3
"""Publish `base_stabilized`: the robot's position, but level.

WHY THIS EXISTS

Turning depth into a LaserScan means filtering points by height above the
floor. That only works in a frame whose z axis points at the sky, and this
robot does not have one.

`odom -> base_footprint` comes from the robot's own state estimator and carries
its FULL attitude -- roll and pitch included. The name is misleading: by
convention base_footprint is the level ground projection, but here it tilts
with the body. Everything below it in the URDF (base_link, pelvis, the camera)
is rigidly attached, so there is no level frame anywhere in the tree.

That matters more than it sounds. The G1 stands with a measured 7.5 degree
forward lean and pitches on every step. A wall point 4 m away sits

    4 * sin(7.5 deg) = 0.52 m

off its true height when measured against a tilted axis. A 0.3-1.5 m wall band
would swing by half a metre, admitting floor at one moment and excluding walls
the next.

WHAT THIS PUBLISHES

    odom -> base_stabilized

Same x and y as the robot, z on the ground plane, and yaw only -- roll and
pitch discarded. So:

    range  = horizontal distance from the robot   (what a scan should measure)
    z      = true height above the floor          (what the filter needs)

Deriving it from /odom rather than un-tilting base_footprint keeps it to one
quaternion-to-yaw conversion, and it stays correct if the URDF changes.

WHY NOT JUST FIX base_footprint

Because the attitude has to enter the tree somewhere. The URDF joins
base_footprint to base_link with a FIXED joint, so if base_footprint were
levelled, base_link would be levelled too -- and base_link is the robot body,
which genuinely does tilt. Levelling it would be wrong in a way that is much
harder to notice. A separate frame keeps both meanings intact.
"""

import math

import rclpy
import rclpy.duration
import rclpy.node
import rclpy.time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSPresetProfiles
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(x, y, z, w):
    """Rotation about the world z axis, discarding roll and pitch."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class BaseStabilizer(rclpy.node.Node):
    def __init__(self):
        super().__init__('base_stabilizer')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('stabilized_frame', 'base_stabilized')
        # The floor is z=0 in odom because odom_bridge forces position.z flat
        # (odom.z tracks pelvis height, which is the robot crouching, not the
        # floor moving). Override if the ground plane sits elsewhere.
        self.declare_parameter('ground_z', 0.0)
        # Date the transform slightly into the future.
        #
        # /odom is stamped at capture, so this frame is always a few ms behind
        # "now". Anything looking it up at the current time -- RViz's 2D Pose
        # Estimate is the one that bit us -- fails with "would require
        # extrapolation into the future", and AMCL DISCARDS the pose with only
        # a log line. The symptom looks like poor localisation: particles
        # never converge and no map -> odom ever appears, which sends you
        # tuning the filter when the pose was never accepted at all.
        #
        # Publishing a stamp slightly ahead keeps the transform valid for
        # lookups at now. The cost is that the frame claims to be this much
        # newer than the data behind it; at walking pace 50 ms is under 5 cm,
        # far below the scan noise it feeds.
        self.declare_parameter('transform_offset', 0.05)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.stabilized_frame = self.get_parameter('stabilized_frame').value
        self.ground_z = float(self.get_parameter('ground_z').value)
        self.offset = float(self.get_parameter('transform_offset').value)

        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._on_odom, QoSPresetProfiles.SENSOR_DATA.value)

        self.count = 0
        self.max_tilt_deg = 0.0
        self.create_timer(10.0, self._report)

        self.get_logger().info(
            f'publishing {self.odom_frame} -> {self.stabilized_frame} '
            f'(level, yaw only, stamped +{self.offset * 1000:.0f} ms)')

    def _on_odom(self, message):
        q = message.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        transform = TransformStamped()
        # The odometry's own stamp, pushed forward by transform_offset so
        # lookups at "now" resolve. See the parameter comment above.
        stamp = rclpy.time.Time.from_msg(message.header.stamp) + \
            rclpy.duration.Duration(seconds=self.offset)
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.stabilized_frame

        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = self.ground_z

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)

        self.broadcaster.sendTransform(transform)

        self.count += 1
        # How much attitude is being discarded, i.e. how wrong a height filter
        # would have been without this frame. Reported so the cost is visible
        # rather than assumed.
        tilt = math.degrees(math.acos(max(-1.0, min(1.0,
            1.0 - 2.0 * (q.x * q.x + q.y * q.y)))))
        self.max_tilt_deg = max(self.max_tilt_deg, tilt)

    def _report(self):
        if self.count == 0:
            self.get_logger().warning(
                'no /odom received -- is odom_bridge running? Without this '
                'frame the scan height filter is meaningless.')
        else:
            self.get_logger().info(
                f'{self.count / 10.0:.0f} Hz; largest tilt removed so far '
                f'{self.max_tilt_deg:.1f} deg')
        self.count = 0


def main(args=None):
    rclpy.init(args=args)
    node = BaseStabilizer()
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

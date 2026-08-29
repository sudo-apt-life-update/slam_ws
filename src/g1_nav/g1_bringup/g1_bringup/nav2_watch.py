#!/usr/bin/env python3
"""Live telemetry during a real Nav2 goal, for diagnosing stuck navigation.

    ros2 run g1_bringup nav2_watch
    ros2 run g1_bringup nav2_watch --duration 300 --out ~/nav2_watch.log

WHY THIS EXISTS

2026-08-25's Nav2 test got stuck: 14 "Failed to make progress" events, ~0.32 m
net movement over ~300 s, despite the robot completing a real ~180 deg turn.
Every attempt to explain it was reconstructed AFTER THE FACT from static log
files -- correlating timestamps across controller_server, collision_monitor
and loco_bridge logs by hand. That found one real bug (a rotation-speed
mismatch, see INSTRUCTIONS.md section 11c) but ran out of runway before
finding why forward WALKING still fails.

Log archaeology has a specific blind spot: it can show THAT commanded
velocity dropped to near zero, but not WHY -- was the path clear and RPP's
own regulation logic throttling it (costmap proximity, curvature), or was
something further down the chain (velocity_smoother, collision_monitor)
overriding a perfectly good command? Those look identical after the fact.
This script samples every stage of that chain AT THE SAME INSTANT, live,
specifically to tell the difference.

The same mistake happened twice in that session for a second reason: a
long-running watcher's LAST KNOWN value was reported as CURRENT state without
re-querying live, producing a false "localisation disconnected from reality"
scare over what was actually just a ~200 s old snapshot. This script always
prints the age of every value alongside it, not just the value, specifically
so a stale read is visible instead of silently indistinguishable from a fresh
one.

WHAT IT CAPTURES, each ~0.5 s tick

    pose        map -> base_stabilized (TF), so you see where the robot
                actually is, not just what it was commanded to do
    cmd chain   /cmd_vel_nav (raw controller output) -> /cmd_vel_smoothed
                (after velocity_smoother) -> /cmd_vel (after collision_monitor)
                Printed side by side. If cmd_vel_nav is healthy and
                cmd_vel_smoothed or cmd_vel is near zero, something
                DOWNSTREAM of the controller is the problem, not RPP itself.
    costmap     cost at the robot's own footprint cell, and distance to the
                nearest cell above OBSTACLE_COST_THRESHOLD within
                COSTMAP_SEARCH_RADIUS. Directly tests the "RPP's regulated
                speed scaling is being throttled by nearby cost" hypothesis
                from INSTRUCTIONS.md section 11c -- if this stays large while
                cmd_vel_nav stays near zero, that hypothesis is dead and the
                real cause is elsewhere.
    plan        length of the current global plan (/plan) and RPP's own copy
                of what it thinks it's tracking (/received_global_plan) --
                if these disagree, the controller may be executing a stale
                plan.
    action      distance_remaining, number_of_recoveries, navigation_time
                from /navigate_to_pose's own feedback, when a goal is active.

HOW TO USE IT

Start this BEFORE sending a goal, watch it live in its own terminal (not
just the saved log), and send the goal from RViz as usual. Every line is
timestamped from this script's own start, not wall-clock, to make the
timeline easy to correlate against the launch log if still needed.
"""

import argparse
import math
import time

import rclpy
import rclpy.node
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformException, TransformListener

# A cell above this is treated as "an obstacle, or close enough to one that
# RPP's cost-regulated speed scaling should be reacting to it". 253 is
# costmap_2d's own INSCRIBED_INFLATED_OBSTACLE -- guaranteed collision if the
# footprint is centred there. Lower than the absolute LETHAL_OBSTACLE (254)
# specifically to catch "close but not yet touching" before it becomes fatal.
OBSTACLE_COST_THRESHOLD = 253
# Local costmap is 4x4 m (nav2_params.yaml) -- searching the whole thing is
# cheap and avoids missing something just outside an arbitrarily chosen
# smaller box.
COSTMAP_SEARCH_RADIUS = 2.0


def quat_to_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Watcher(rclpy.node.Node):
    def __init__(self, out_path):
        super().__init__('nav2_watch')
        self.buffer = Buffer()
        TransformListener(self.buffer, self)

        self.latest = {}  # topic -> (value, monotonic time received)

        def track(name, msg_type, extract):
            def cb(msg, name=name, extract=extract):
                self.latest[name] = (extract(msg), time.monotonic())
            self.create_subscription(msg_type, name, cb, 10)

        track('/cmd_vel_nav', Twist, lambda m: (m.linear.x, m.linear.y, m.angular.z))
        track('/cmd_vel_smoothed', Twist,
              lambda m: (m.linear.x, m.linear.y, m.angular.z))
        track('/cmd_vel', Twist, lambda m: (m.linear.x, m.linear.y, m.angular.z))
        # costmap_raw, NOT the plain /local_costmap/costmap OccupancyGrid --
        # nav2_costmap_2d RESCALES cost to a 0-100 occupancy-probability
        # range for that one (253 -> 99, 254 -> 100, see
        # costmap_2d_publisher.cpp's cost_translation_table_), which silently
        # breaks the raw 253/254 obstacle thresholds used below. costmap_raw
        # (nav2_msgs/msg/Costmap) carries the genuine unscaled 0-255 values.
        track('/local_costmap/costmap_raw', Costmap, lambda m: m)
        track('/plan', Path, lambda m: len(m.poses))
        track('/received_global_plan', Path, lambda m: len(m.poses))
        track('/navigate_to_pose/_action/status', GoalStatusArray,
              lambda m: self._status_text(m))

        self.feedback = None
        # Subscribing to the raw feedback topic, not going through an
        # ActionClient -- this script did not send the goal, RViz did, and
        # there is no goal handle to attach a client-side feedback callback
        # to. The topic is there regardless of who's driving.
        from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
        self.create_subscription(
            NavigateToPose_FeedbackMessage, '/navigate_to_pose/_action/feedback',
            self._on_feedback, 10)

        self.out = open(out_path, 'a')
        self.get_logger().info(f'writing to {out_path}')

    def _on_feedback(self, msg):
        self.feedback = (msg.feedback, time.monotonic())

    @staticmethod
    def _status_text(msg):
        if not msg.status_list:
            return 'NONE'
        names = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
                 4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        return names.get(msg.status_list[-1].status, str(msg.status_list[-1].status))

    def _get(self, name):
        """(value, age_seconds) or (None, None) if never received."""
        entry = self.latest.get(name)
        if entry is None:
            return None, None
        value, t = entry
        return value, time.monotonic() - t

    def _costmap_probe(self, robot_x, robot_y):
        grid, age = self._get('/local_costmap/costmap_raw')
        if grid is None:
            return None, None, age
        res = grid.metadata.resolution
        ox = grid.metadata.origin.position.x
        oy = grid.metadata.origin.position.y
        w, h = grid.metadata.size_x, grid.metadata.size_y
        gx = int((robot_x - ox) / res)
        gy = int((robot_y - oy) / res)
        own_cost = None
        if 0 <= gx < w and 0 <= gy < h:
            own_cost = grid.data[gy * w + gx]

        search_cells = int(COSTMAP_SEARCH_RADIUS / res)
        nearest = None
        for dy in range(-search_cells, search_cells + 1):
            cy = gy + dy
            if not (0 <= cy < h):
                continue
            for dx in range(-search_cells, search_cells + 1):
                cx = gx + dx
                if not (0 <= cx < w):
                    continue
                if grid.data[cy * w + cx] >= OBSTACLE_COST_THRESHOLD:
                    d = math.hypot(dx * res, dy * res)
                    if nearest is None or d < nearest:
                        nearest = d
        return own_cost, nearest, age

    def tick(self, t0):
        try:
            tf = self.buffer.lookup_transform(
                'map', 'base_stabilized', rclpy.time.Time())
            t = tf.transform.translation
            yaw = math.degrees(quat_to_yaw(tf.transform.rotation))
            pose_str = f'pose=({t.x:5.2f},{t.y:5.2f}) yaw={yaw:6.1f}deg'
            x, y = t.x, t.y
        except TransformException as exc:
            pose_str = f'pose=NO_TF({exc})'
            x = y = None

        parts = [f'{time.monotonic() - t0:6.1f}s', pose_str]

        for label, topic in [('nav', '/cmd_vel_nav'), ('smooth', '/cmd_vel_smoothed'),
                             ('final', '/cmd_vel')]:
            v, age = self._get(topic)
            if v is None:
                parts.append(f'{label}=NEVER')
            else:
                vx, vy, wz = v
                stale = '!' if age is not None and age > 1.0 else ''
                parts.append(f'{label}=(vx{vx:+.2f},vy{vy:+.2f},wz{wz:+.2f}){stale}')

        if x is not None:
            own_cost, nearest, cm_age = self._costmap_probe(x, y)
            if own_cost is None:
                parts.append('cost=NO_COSTMAP')
            else:
                stale = '!' if cm_age is not None and cm_age > 2.0 else ''
                near_str = f'{nearest:.2f}m' if nearest is not None else '>2m clear'
                parts.append(f'cost=own:{own_cost} nearest_obstacle:{near_str}{stale}')

        plan_len, plan_age = self._get('/plan')
        rpp_len, rpp_age = self._get('/received_global_plan')
        parts.append(f'plan=global:{plan_len if plan_len is not None else "-"} '
                     f'rpp_view:{rpp_len if rpp_len is not None else "-"}')

        status, status_age = self._get('/navigate_to_pose/_action/status')
        parts.append(f'status={status if status else "NONE"}')

        if self.feedback is not None:
            fb, fb_age = self.feedback
            stale = '!' if fb_age > 2.0 else ''
            parts.append(
                f'remaining={fb.distance_remaining:.2f}m '
                f'recoveries={fb.number_of_recoveries}{stale}')

        line = '  '.join(parts)
        print(line)
        self.out.write(line + '\n')
        self.out.flush()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=300.0,
                        help='seconds to watch (default 300)')
    parser.add_argument('--out', type=str,
                        default='/tmp/nav2_watch.log',
                        help='where to also save the log')
    parser.add_argument('--rate', type=float, default=2.0,
                        help='samples per second (default 2)')
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Watcher(args.out)
    node.get_logger().info(
        f'watching for {args.duration:.0f} s at {args.rate:.1f} Hz -- '
        f'send a goal now (RViz "2D Goal Pose") and watch this terminal')

    t0 = time.monotonic()
    period = 1.0 / args.rate
    try:
        while time.monotonic() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=period / 2)
            node.tick(t0)
            time.sleep(period / 2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())

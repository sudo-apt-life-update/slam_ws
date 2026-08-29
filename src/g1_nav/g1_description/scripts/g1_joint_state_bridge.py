#!/usr/bin/env python3
"""
g1_joint_state_bridge — UDP (from g1_lowstate_reader) -> ROS 2 /joint_states

Why this exists
---------------
The G1 model in RViz needs joint angles to animate its legs/arms. The real
robot's joint angles live on the Unitree "rt/lowstate" DDS topic, but unitree
DDS and ROS 2 DDS cannot share a process (the libddsc clash). So the host-side
g1_lowstate_reader (unitree-DDS only) reads lowstate and streams the 29 joint
angles over UDP; this node (ROS-only) receives them and republishes as
sensor_msgs/JointState on /joint_states.

robot_state_publisher then turns /joint_states into TF, so the model's limbs
move in RViz exactly as the real robot walks.

This REPLACES joint_state_publisher (which only publishes zeros). Don't run both
at once -- they would fight over /joint_states. Use robot_on_map.launch.py with
joints:=live, which runs this instead of joint_state_publisher.

Run (inside the Humble container, after sourcing /ws/install/setup.bash):
  ros2 run g1_description g1_joint_state_bridge.py
  # or:  ros2 launch g1_description robot_on_map.launch.py joints:=live
Then start the host reader:
  ~/unitree_localization/g1_lowstate_reader/run_lowstate_reader.sh --network eno1
"""

import socket
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ---- must match g1_lowstate_reader.cpp ----
UDP_HOST = "127.0.0.1"
UDP_PORT = 8898
MAGIC = 0x6A6F696E          # 'join'
NUM_JOINTS = 29
PUBLISH_HZ = 50.0

# URDF joint order == Unitree G1 motor index order (0..28). Verified against
# g1_description/urdf/g1_29dof.urdf and the SDK's JointIndex enum.
JOINT_NAMES = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
assert len(JOINT_NAMES) == NUM_JOINTS

_PACKET_FMT = "<I%df" % NUM_JOINTS           # magic (uint32) + 29 float32
_PACKET_SIZE = struct.calcsize(_PACKET_FMT)  # 4 + 116 = 120 bytes


class JointStateBridge(Node):
    def __init__(self):
        super().__init__("g1_joint_state_bridge")

        # latest joint angles -- zeros (nominal standing) until the first packet
        self.positions = [0.0] * NUM_JOINTS
        self.got_data = False

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind((UDP_HOST, UDP_PORT))

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0 / PUBLISH_HZ, self.on_timer)

        self.get_logger().info(
            f"bridge up: udp {UDP_HOST}:{UDP_PORT} -> /joint_states "
            f"({NUM_JOINTS} joints @ {PUBLISH_HZ:.0f} Hz). "
            f"Run g1_lowstate_reader on the host to feed it."
        )

    def on_timer(self):
        # drain the socket -> keep only the most recent valid packet
        while True:
            try:
                data = self.sock.recv(4096)
            except BlockingIOError:
                break
            if len(data) != _PACKET_SIZE:
                continue
            fields = struct.unpack(_PACKET_FMT, data)
            if fields[0] != MAGIC:
                continue
            self.positions = list(fields[1:])
            if not self.got_data:
                self.got_data = True
                self.get_logger().info("receiving live joint angles from the robot.")

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = self.positions
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

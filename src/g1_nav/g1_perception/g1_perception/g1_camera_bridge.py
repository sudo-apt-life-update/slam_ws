#!/usr/bin/env python3
"""Republish the G1 head D435 stream from Unitree's image_server as ROS 2 topics.

image_server runs on the robot and publishes one pickled dict per frame over
ZMQ:

    image       JPEG 2560x720  -- see the stitching note below
    depth_raw   uint16 1280x720, millimetres, ALREADY ALIGNED TO COLOUR
    intrinsics  fx, fy, cx, cy, width, height, depth_scale  (colour module)
    timestamp   float, epoch seconds, capture time
    frame_id    int, increments per frame

[STITCHING] The 2560x720 JPEG is `cv2.hconcat([colour, depth_colormap])`, from
ImageServer._capture_frames. So the LEFT half is the real colour image and the
RIGHT half is a turbo/jet *rendering* of the same depth data we already receive
as depth_raw -- decorative for teleop, useless to us, and it doubles the JPEG.
Do not be fooled by measuring which half correlates with depth: the right half
correlates almost perfectly because it *is* the depth.

Alignment comes from the server's `rs.align(rs.stream.color)`, and the
intrinsics from `profile.get_stream(rs.stream.color)` -- so they are the colour
module's, and fx=911 @1280x720 (70 deg HFOV) matches the D435 colour module's
69 deg rather than the depth module's 87 deg. Both the colour image and the
aligned depth therefore share the colour intrinsics and the colour optical
frame.

[CAUTION] The server applies spatial + temporal + hole_filling filters to depth
before sending. Hole filling is why depth arrives ~99.9% dense; it invents
values in occluded regions. That flatters the "valid pixels" check and may not
be what you want for mapping.

Publishes:
    /camera/color/image_raw                     Image      bgr8
    /camera/color/camera_info                   CameraInfo
    /camera/aligned_depth_to_color/image_raw    Image      16UC1, millimetres
    /camera/aligned_depth_to_color/camera_info  CameraInfo

All four share one timestamp per frame -- RTAB-Map and depth_image_proc will not
sync otherwise.
"""

import pickle
import threading
import time

import cv2
import numpy as np
import rclpy
import zmq
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSPresetProfiles,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image


class G1CameraBridge(Node):

    def __init__(self):
        super().__init__('g1_camera_bridge')

        self.declare_parameter('server_address', '192.168.123.164')
        self.declare_parameter('port', 5556)
        self.declare_parameter('optical_frame', 'camera_color_optical_frame')
        # Which half of the stitched JPEG carries real colour. image_server
        # builds it as hconcat([colour, depth_colormap]), so it is the LEFT
        # half. Parameterised in case the server's frame order changes, or the
        # wrist cameras are enabled and append further panels.
        self.declare_parameter('color_half', 'left')
        # The robot's clock tracks this PC to within ~60 ms, most of which is
        # the 1.96 MB transfer itself, so the capture timestamp beats
        # restamping on arrival. Set false if the robot's clock drifts.
        self.declare_parameter('use_server_timestamp', True)
        # Reliable rather than the usual SENSOR_DATA best-effort. A 1280x720
        # uint16 depth frame is 1.84 MB, which DDS splits into many fragments;
        # losing any one drops the whole sample, and measurement showed 8% of
        # depth frames vanishing while CameraInfo (tiny) arrived 100%. A deeper
        # subscriber queue does not help -- the loss is in fragmentation, not
        # backpressure. At 7-30 Hz on one host, reliable costs nothing.
        # It is also strictly more compatible: a reliable publisher can feed a
        # best-effort subscriber, but not the reverse.
        self.declare_parameter('reliable_qos', True)

        address = self.get_parameter('server_address').value
        port = int(self.get_parameter('port').value)
        self.endpoint = f'tcp://{address}:{port}'

        self.optical_frame = self.get_parameter('optical_frame').value
        self.color_half = self.get_parameter('color_half').value
        self.use_server_stamp = bool(
            self.get_parameter('use_server_timestamp').value)

        self.bridge = CvBridge()

        if bool(self.get_parameter('reliable_qos').value):
            sensor_qos = QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            )
        else:
            sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

        self.color_pub = self.create_publisher(
            Image, '/camera/color/image_raw', sensor_qos)
        self.color_info_pub = self.create_publisher(
            CameraInfo, '/camera/color/camera_info', sensor_qos)
        self.depth_pub = self.create_publisher(
            Image, '/camera/aligned_depth_to_color/image_raw', sensor_qos)
        self.depth_info_pub = self.create_publisher(
            CameraInfo, '/camera/aligned_depth_to_color/camera_info',
            sensor_qos)

        self.frames = 0
        self.dropped = 0
        self.last_frame_id = None
        self.logged_layout = False
        self.warned_depth_scale = False

        # How long to tolerate silence before rebuilding the socket. Long
        # enough that a slow server start or a brief network blip does not
        # trigger it; short enough that a session is not lost to it.
        self.declare_parameter('reconnect_after', 12.0)
        self.reconnect_after = float(
            self.get_parameter('reconnect_after').value)
        self.last_frame = time.monotonic()
        self.reconnects = 0

        self.ctx = zmq.Context()
        self.sock = None
        self._connect()

        # A blocking poll on its own thread. The previous implementation used a
        # 1 ms timer, which spins a core to no purpose on a ~8-30 Hz stream.
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()

        self.create_timer(5.0, self._report)

        self.get_logger().info(
            f'listening on {self.endpoint} -> /camera/color/* and '
            f'/camera/aligned_depth_to_color/*')

    # ------------------------------------------------------------------
    # receive
    # ------------------------------------------------------------------

    def _connect(self):
        """Create the subscriber socket, replacing any existing one."""
        if self.sock is not None:
            self.sock.close(linger=0)
        self.sock = self.ctx.socket(zmq.SUB)
        # Always take the newest frame. A queued backlog of stale images is
        # worse than none on a 30 Hz stream.
        self.sock.setsockopt(zmq.CONFLATE, 1)
        self.sock.setsockopt(zmq.RCVHWM, 1)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.setsockopt_string(zmq.SUBSCRIBE, '')
        self.sock.connect(self.endpoint)
        self.last_frame = time.monotonic()

    def _receive_loop(self):
        while self.running:
            # See g1_state_bridge: rclpy's context goes away on Ctrl-C while
            # this thread is still running, and publishing after that raises
            # RCLError. Exit quietly rather than logging a shutdown as a fault.
            if not rclpy.ok():
                break
            try:
                if self.sock.poll(500) == 0:
                    # ZMQ reconnects a dropped TCP session by itself, but it
                    # does not always recover when the PEER REBOOTS: the old
                    # connection is left half-open and the subscriber sits
                    # silent forever while frames are demonstrably on the wire.
                    # Observed after a robot reboot -- a fresh subscriber
                    # received fine while this one warned indefinitely.
                    #
                    # Rebuilding the socket is cheap and recovers in seconds.
                    silence = time.monotonic() - self.last_frame
                    if silence > self.reconnect_after:
                        self.reconnects += 1
                        self.get_logger().warning(
                            f'no frames for {silence:.0f} s -- rebuilding the '
                            f'socket to {self.endpoint} '
                            f'(reconnect #{self.reconnects})')
                        self._connect()
                    continue
                message = self.sock.recv()
                self.last_frame = time.monotonic()
            except zmq.ZMQError:
                break

            try:
                data = pickle.loads(message)
            except Exception as exc:  # noqa: BLE001 - never kill the thread
                self.get_logger().warning(f'undecodable packet: {exc}')
                continue

            if not isinstance(data, dict):
                self.get_logger().warning('packet is not a dict')
                continue

            try:
                self._publish(data)
            except Exception as exc:  # noqa: BLE001
                if not rclpy.ok():
                    break             # shutting down, not a frame problem
                self.get_logger().warning(f'could not publish frame: {exc}')

    # ------------------------------------------------------------------
    # decode
    # ------------------------------------------------------------------

    def _decode_depth(self, raw):
        """Depth may arrive as a bare array or compressed.

        image_server originally pickled the uint16 array whole, which is 1.84 MB
        and caps the stream near 7.5 Hz. Compressed forms are accepted here so
        the ROS side needs no change when the robot starts compressing.
        """
        if raw is None:
            return None

        if isinstance(raw, np.ndarray):
            depth = raw
        elif isinstance(raw, (bytes, bytearray)):
            buf = np.frombuffer(raw, dtype=np.uint8)
            depth = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)  # PNG-16
            if depth is None:
                try:
                    import zstandard
                    decompressed = zstandard.ZstdDecompressor().decompress(
                        bytes(raw))
                    depth = np.frombuffer(
                        decompressed, dtype=np.uint16).reshape(720, 1280)
                except Exception as exc:  # noqa: BLE001
                    raise ValueError(
                        f'depth is bytes but decodes as neither PNG nor '
                        f'zstd: {exc}')
        else:
            raise ValueError(f'unexpected depth type {type(raw).__name__}')

        if depth.dtype != np.uint16:
            depth = depth.astype(np.uint16)
        return depth

    def _camera_info(self, stamp, intrinsics, width, height):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.optical_frame
        info.width = int(intrinsics.get('width', width))
        info.height = int(intrinsics.get('height', height))

        fx = float(intrinsics['fx'])
        fy = float(intrinsics['fy'])
        cx = float(intrinsics['cx'])
        cy = float(intrinsics['cy'])

        # image_server sends no distortion coefficients. RealSense colour is
        # delivered already rectified, so zeros are correct rather than merely
        # convenient -- but if a calibration ever shows otherwise, this is the
        # place to fix it.
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    # ------------------------------------------------------------------
    # publish
    # ------------------------------------------------------------------

    def _publish(self, data):
        jpeg = data.get('image') or data.get('head_image') or data.get('rgb')
        if jpeg is None:
            raise ValueError('packet carries no image')

        stitched = cv2.imdecode(
            np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if stitched is None:
            raise ValueError('JPEG failed to decode')

        depth = self._decode_depth(data.get('depth_raw'))
        if depth is None:
            raise ValueError('packet carries no depth')

        height, width = stitched.shape[:2]
        depth_h, depth_w = depth.shape[:2]

        # The stitched frame is two images side by side; each half should match
        # the depth width. If it ever arrives unstitched, use it whole.
        if width == 2 * depth_w:
            mid = width // 2
            color = (stitched[:, mid:] if self.color_half == 'right'
                     else stitched[:, :mid])
        else:
            color = stitched

        if color.shape[:2] != depth.shape[:2]:
            raise ValueError(
                f'colour {color.shape[1]}x{color.shape[0]} does not match '
                f'depth {depth_w}x{depth_h}; depth is supposed to be aligned '
                f'to colour')

        intrinsics = data.get('intrinsics')
        if not intrinsics:
            raise ValueError(
                'packet carries no intrinsics -- CameraInfo cannot be built, '
                'and RTAB-Map needs it')

        # ROS convention for 16UC1 depth is millimetres. RealSense reports its
        # unit via depth_scale (0.001 = mm), so rescale rather than trusting it.
        depth_scale = float(intrinsics.get('depth_scale', 0.001))
        if abs(depth_scale - 0.001) > 1e-9:
            if not self.warned_depth_scale:
                self.warned_depth_scale = True
                self.get_logger().warning(
                    f'depth_scale is {depth_scale}, not 0.001; rescaling to '
                    f'millimetres')
            depth = np.clip(
                depth.astype(np.float32) * depth_scale * 1000.0,
                0, 65535).astype(np.uint16)

        if self.use_server_stamp and 'timestamp' in data:
            stamp = rclpy.time.Time(
                nanoseconds=int(float(data['timestamp']) * 1e9)).to_msg()
        else:
            stamp = self.get_clock().now().to_msg()

        frame_id = data.get('frame_id')
        if frame_id is not None and self.last_frame_id is not None:
            gap = frame_id - self.last_frame_id
            if gap > 1:
                self.dropped += gap - 1
        if frame_id is not None:
            self.last_frame_id = frame_id

        if not self.logged_layout:
            self.logged_layout = True
            layout = (f'stitched {width}x{height}, took the '
                      f'{self.color_half} half'
                      if width == 2 * depth_w else
                      f'single panel {width}x{height}')
            self.get_logger().info(
                f'{layout} -> colour {color.shape[1]}x{color.shape[0]}, '
                f'depth {depth_w}x{depth_h}, '
                f'fx={intrinsics["fx"]:.1f} cx={intrinsics["cx"]:.1f}, '
                f'frame {self.optical_frame}')

        info = self._camera_info(
            stamp, intrinsics, color.shape[1], color.shape[0])

        color_msg = self.bridge.cv2_to_imgmsg(color, encoding='bgr8')
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = self.optical_frame

        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='16UC1')
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self.optical_frame

        self.color_pub.publish(color_msg)
        self.color_info_pub.publish(info)
        self.depth_pub.publish(depth_msg)
        self.depth_info_pub.publish(info)

        self.frames += 1

    def _report(self):
        if self.frames == 0:
            self.get_logger().warning(
                f'no frames from {self.endpoint} in 5 s -- is image_server '
                f'running on the robot?')
        else:
            message = f'{self.frames / 5.0:.1f} Hz'
            if self.dropped:
                message += f', {self.dropped} frames dropped in transit'
            if self.reconnects:
                message += f', {self.reconnects} socket rebuild(s)'
            self.get_logger().info(message)
        self.frames = 0
        self.dropped = 0

    def destroy_node(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.sock.close()
        self.ctx.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = G1CameraBridge()
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

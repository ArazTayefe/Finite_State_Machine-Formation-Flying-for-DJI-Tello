#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Image

import threading, time, logging, cv2
from cv_bridge  import CvBridge
from djitellopy import TelloSwarm


class TelloCommunicationNode(Node):
    BASE_VS_PORT = 11111            # 11111 / 11121 / 11131 …

    def __init__(self):
        super().__init__('tello_communication_node')

        # ─── Swarm set-up ────────────────────────────────────────────────
        self.names = ['Alpha', 'Bravo', 'Charlie']
        ips        = ['10.42.0.95', '10.42.0.97', '10.42.0.204']
        self.swarm = TelloSwarm.fromIps(ips)
        self.swarm.connect()

        self.bridge     = CvBridge()
        self.image_pubs = {}

        # Image QoS – reliable, depth 10 (matches rviz2 / showimage defaults)
        img_qos = QoSProfile(
            depth       = 10,
            reliability = ReliabilityPolicy.RELIABLE,
            durability  = DurabilityPolicy.VOLATILE)

        def per_drone(i, tello):
            port = self.BASE_VS_PORT + i*10
            tello.streamoff()
            tello.change_vs_udp(port)
            tello.streamon()
            time.sleep(0.5)                        # let first key-frame arrive
            # tello.takeoff()

            # rc placeholders
            tello.rc_control_x = tello.rc_control_y = \
            tello.rc_control_z = tello.rc_control_yaw = 0

            # ROS publisher & capture thread
            name  = self.names[i]
            topic = f'{name}/image_raw'
            self.image_pubs[name] = self.create_publisher(Image, topic, img_qos)
            threading.Thread(target=self._video_loop,
                             args=(name, port),
                             daemon=True).start()

        self.swarm.parallel(per_drone)
        self.drones = {n: self.swarm.tellos[i] for i, n in enumerate(self.names)}

        # ─── Command & landing topics ───────────────────────────────────
        for drone in self.names:
            for axis in ['Vel_x', 'Vel_y', 'Vel_z', 'AngVel_yaw']:
                self.create_subscription(
                    Float64, f'{drone}_cmd_{axis}',
                    self._vel_cb(drone, axis), 10)

            self.create_subscription(
                Bool, f'{drone}_land',
                self._land_cb(drone), 10)

        logging.getLogger('djitellopy').setLevel(logging.WARNING)
        rclpy.get_default_context().on_shutdown(self.on_shutdown)
        self.get_logger().info('TelloCommunicationNode initialised.')

    # ─── Video grab & publish ───────────────────────────────────────────
    def _video_loop(self, name: str, port: int):
        url = (f'udp://@0.0.0.0:{port}'
               '?overrun_nonfatal=1&fifo_size=5000000&listen=1')
        self.get_logger().info(f'[{name}] opening video stream: {url}')
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        pub      = self.image_pubs[name]
        got_frame = False
        start_t   = self.get_clock().now().nanoseconds

        while rclpy.ok():
            ok, frame = cap.read()
            if not ok:
                # after 5 s without a frame, complain once
                if (not got_frame and
                    self.get_clock().now().nanoseconds - start_t > 5e9):
                    self.get_logger().error(
                        f'[{name}] No video packets received on UDP {port}. '
                        'Check:\n  • Wi-Fi NIC / IP routing\n  • Firewall\n  • '
                        'Drone really streaming (ffplay udp://@0.0.0.0:PORT)')
                    got_frame = True  # prevent spamming
                time.sleep(0.02)
                continue

            got_frame = True
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)          # NEW
            msg   = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')  # CHANGED
            msg.header.stamp = self.get_clock().now().to_msg()
            pub.publish(msg)

    # ─── Velocity / landing helpers ─────────────────────────────────────
    def _vel_cb(self, name, axis):
        return lambda msg: self._handle_vel(msg, name, axis)

    def _handle_vel(self, msg, name, axis):
        d, sc, lim = self.drones[name], 100.0, 100
        v = max(min(msg.data * sc, lim), -lim)
        if   axis == 'Vel_x':        d.rc_control_x   = v
        elif axis == 'Vel_y':        d.rc_control_y   = v
        elif axis == 'Vel_z':        d.rc_control_z   = v
        elif axis == 'AngVel_yaw':   d.rc_control_yaw = v

        threading.Thread(target=lambda:
            d.send_rc_control(int(d.rc_control_x),
                              int(d.rc_control_y),
                              int(d.rc_control_z),
                              int(d.rc_control_yaw)),
            daemon=True).start()

    def _land_cb(self, name):
        return lambda msg: self._safe_land(name) if msg.data else None

    def _safe_land(self, name, tries=3):
        d = self.drones[name]
        for _ in range(3): d.send_rc_control(0,0,0,0); time.sleep(0.3)
        for i in range(tries):
            try: d.land(); return
            except Exception as e:
                self.get_logger().warn(f'[{name}] land retry {i+1}: {e}')
                time.sleep(1.5*(i+1))
        self.get_logger().error(f'[{name}] FAILED to land.')

    # ─── Shutdown ──────────────────────────────────────────────────────
    def on_shutdown(self):
        self.get_logger().info('Landing swarm…')
        self.swarm.sequential(lambda i, t: t.land())
        self.swarm.end()
        self.get_logger().info('Cleanup complete.')


# ─── main ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = TelloCommunicationNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

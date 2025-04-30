#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node       import Node
from rclpy.qos        import qos_profile_sensor_data
from std_msgs.msg     import Float64, Bool
from sensor_msgs.msg  import Image

import threading, time, logging, cv2
from cv_bridge        import CvBridge
from djitellopy       import TelloSwarm


class TelloCommunicationNode(Node):
    """
    One ROS 2 node that:
      • builds a TelloSwarm from the given IP list
      • changes the video-stream UDP port of each drone (so streams don’t clash)
      • publishes velocity / landing commands (as before)
      • publishes each camera feed on  <drone>/image_raw   (sensor_msgs/Image)
    """
    BASE_VS_PORT = 11111          # first UDP port; each next drone gets +10

    # ──────────────────────────────────────────────────────────────────────────
    def __init__(self):
        super().__init__('tello_communication_node')

        # ─── Swarm initialisation ───────────────────────────────────────────
        self.names = ['Alpha', 'Bravo', 'Charlie']
        ips        = ['10.42.0.95', '10.42.0.97', '10.42.0.204']

        self.swarm  = TelloSwarm.fromIps(ips)
        self.swarm.connect()                                   # broadcast connect
        self.bridge = CvBridge()
        self.image_pubs = {}                                   # name -> publisher

        def per_drone_setup(i: int, tello):
            """Runs in parallel for every Tello during initialisation"""
            port = self.BASE_VS_PORT + i * 10                  # 11111 / 11121 / 11131 …
            tello.streamoff()
            tello.change_vs_udp(port)                          # custom port
            tello.streamon()                                   # turn camera back on
            tello.takeoff()

            # attach RC placeholders so we can mutate them later
            tello.rc_control_x = tello.rc_control_y = \
            tello.rc_control_z = tello.rc_control_yaw = 0

            # make ROS publisher & start capture thread
            name  = self.names[i]
            topic = f'{name}/image_raw'
            self.image_pubs[name] = self.create_publisher(
                                        Image, topic,
                                        qos_profile_sensor_data)
            threading.Thread(target=self._video_loop,
                             args=(name, port),
                             daemon=True).start()

        self.swarm.parallel(per_drone_setup)                   # run setup on all drones
        self.drones = {n: self.swarm.tellos[i]                 # handy lookup
                       for i, n in enumerate(self.names)}

        # ─── ROS 2 subscriptions (same topics as before) ────────────────────
        for drone in self.names:
            for axis in ['Vel_x', 'Vel_y', 'Vel_z', 'AngVel_yaw']:
                topic = f'{drone}_cmd_{axis}'
                self.create_subscription(Float64, topic,
                                         self._make_velocity_cb(drone, axis), 10)

            land_topic = f'{drone}_land'
            self.create_subscription(Bool, land_topic,
                                     self._make_land_cb(drone), 10)

        logging.getLogger('djitellopy').setLevel(logging.WARNING)
        rclpy.get_default_context().on_shutdown(self.on_shutdown)
        self.get_logger().info('TelloCommunicationNode initialised.')

    # ─── Video capture worker ────────────────────────────────────────────────
    def _video_loop(self, name: str, port: int):
        """
        Opens an ffmpeg-backed UDP stream and publishes frames until shutdown.
        Runs in its own daemon thread per drone.
        """
        self.get_logger().info(f'[{name}] opening video stream on UDP {port}')
        cap = cv2.VideoCapture(f'udp://0.0.0.0:{port}', cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)            # keep latency low
        pub = self.image_pubs[name]

        while cap.isOpened() and rclpy.ok():
            ok, frame = cap.read()
            if not ok:
                self.get_logger().warn(f'[{name}] dropped frame'); continue
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            pub.publish(msg)
        cap.release()
        self.get_logger().info(f'[{name}] video thread ended.')

    # ─── Velocity handling ───────────────────────────────────────────────────
    def _make_velocity_cb(self, name, axis):
        return lambda msg: self._handle_velocity(msg, name, axis)

    def _handle_velocity(self, msg, name, axis):
        drone = self.drones[name]
        scale, limit = 100.0, 100
        val = max(min(msg.data * scale, limit), -limit)

        if   axis == 'Vel_x':        drone.rc_control_x   = val
        elif axis == 'Vel_y':        drone.rc_control_y   = val
        elif axis == 'Vel_z':        drone.rc_control_z   = val
        elif axis == 'AngVel_yaw':   drone.rc_control_yaw = val

        threading.Thread(target=lambda:
            drone.send_rc_control(int(drone.rc_control_x),
                                  int(drone.rc_control_y),
                                  int(drone.rc_control_z),
                                  int(drone.rc_control_yaw)),
            daemon=True).start()

    # ─── Landing handling ────────────────────────────────────────────────────
    def _make_land_cb(self, name):
        return lambda msg: self._handle_land(msg, name)

    def _handle_land(self, msg, name):
        if msg.data:
            self.get_logger().info(f'[{name}] Landing…')
            threading.Thread(target=self._safe_land, args=(name,),
                             daemon=True).start()

    def _safe_land(self, name, retries=3):
        drone = self.drones[name]
        for _ in range(3):                               # stabilise first
            drone.send_rc_control(0, 0, 0, 0); time.sleep(0.3)
        for i in range(retries):
            try:
                drone.land(); return
            except Exception as e:
                self.get_logger().warn(f'[{name}] land retry {i+1}: {e}')
                time.sleep(1.5 * (i + 1))
        self.get_logger().error(f'[{name}] FAILED to land – manual intervention!')

    # ─── Shutdown ────────────────────────────────────────────────────────────
    def on_shutdown(self):
        self.get_logger().info('ROS 2 shutdown – landing swarm…')
        self.swarm.sequential(lambda i, t: t.land())
        self.swarm.end()
        self.get_logger().info('All drones down; cleanup complete.')


# ─── main ────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = TelloCommunicationNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():                                  # if ctrl-C not hit twice
            node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

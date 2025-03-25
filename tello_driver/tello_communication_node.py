import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from djitellopy import Tello
import threading
import time
import cv2
import logging
from std_msgs.msg import Bool

class TelloCommunicationNode(Node):
    def __init__(self):
        super().__init__('tello_communication_node')
        self.drones = {
            'Alpha': Tello('10.42.0.95'),
            'Bravo': Tello('10.42.0.97'),
            'Charlie': Tello('10.42.0.204')
        }
        # self.drones = {'Alpha': Tello('10.42.0.95')}
        self.bridge = CvBridge()
        self.initialize_drones()

        for drone_name in self.drones:
            for axis in ['Vel_x', 'Vel_y', 'Vel_z', 'AngVel_yaw']:
                topic_name = f'{drone_name}_cmd_{axis}'
                self.get_logger().info(f'Subscribing to topic: {topic_name}')
                self.create_subscription(Float64, topic_name, 
                                         self.create_velocity_callback(drone_name, axis), 10)

        for drone_name in self.drones:
            land_topic = f'{drone_name}_land'
            self.get_logger().info(f'Subscribing to topic: {land_topic}')
            self.create_subscription(Bool, land_topic, self.create_landing_callback(drone_name), 10)
            
        rclpy.get_default_context().on_shutdown(self.on_shutdown)
        threading.Thread(target=self.shutdown_listener).start()
        logging.getLogger('djitellopy').setLevel(logging.WARNING)

    def initialize_drones(self):
        threads = [threading.Thread(target=self.setup_drone, args=(drone_name,)) for drone_name in self.drones]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

    def setup_drone(self, drone_name):
        drone = self.drones[drone_name]
        drone.connect()
        drone.streamoff()
        drone.takeoff()

        # Initialize rc controls
        drone.rc_control_x = 0
        drone.rc_control_y = 0
        drone.rc_control_z = 0
        drone.rc_control_yaw = 0

    def send_velocity_command(self, drone):
        #self.get_logger().info(f'Sending velocity command: vx={drone.rc_control_x}, vy={drone.rc_control_y}, vz={drone.rc_control_z}, yaw={drone.rc_control_yaw}')
        drone.send_rc_control(int(drone.rc_control_x), int(drone.rc_control_y), int(drone.rc_control_z), int(drone.rc_control_yaw))

    def handle_velocity(self, msg, drone_name, axis):
        #self.get_logger().info(f'Received velocity message: {msg.data} for drone: {drone_name}, axis: {axis}')
        drone = self.drones[drone_name]

        scale_factor = 100.0  # Scale the velocity commands to be within -100 to 100
        limit = 100
        if axis == 'Vel_x':
            drone.rc_control_x = max(min(msg.data * scale_factor, limit), -limit)
        elif axis == 'Vel_y':
            drone.rc_control_y = max(min(msg.data * scale_factor, limit), -limit)
        elif axis == 'Vel_z':
            drone.rc_control_z = max(min(msg.data * scale_factor, limit), -limit)
        elif axis == 'AngVel_yaw':
            drone.rc_control_yaw = max(min(msg.data * scale_factor, limit), -limit)

        threading.Thread(target=self.send_velocity_command, args=(drone,)).start()

    def create_velocity_callback(self, drone_name, axis):
        return lambda msg: self.handle_velocity(msg, drone_name, axis)

    def land_all_drones(self):
        self.get_logger().info('Landing all drones now...')
        for drone_name in self.drones:
            success = self.safe_land(drone_name)
            if not success:
                self.get_logger().warn(f"[{drone_name}] Manual intervention may be required.")

    def create_landing_callback(self, drone_name):
        return lambda msg: self.handle_landing(msg, drone_name)

    def handle_landing(self, msg, drone_name):
        if msg.data:
            self.get_logger().info(f"[{drone_name}] Landing command received. Initiating safe landing...")
            threading.Thread(target=self.safe_land, args=(drone_name,)).start()

    def safe_land(self, drone_name, max_retries=3):
        drone = self.drones[drone_name]
        self.get_logger().info(f"[{drone_name}] Initiating safe landing...")
    
        try:
            # Step 1: Stabilize with rc 0 0 0 0
            for _ in range(3):
                drone.send_rc_control(0, 0, 0, 0)
                time.sleep(0.3)
    
            # Step 2: Retry landing with exponential backoff
            for i in range(max_retries):
                try:
                    self.get_logger().info(f"[{drone_name}] Sending land command (attempt {i+1})...")
                    drone.land()
                    self.get_logger().info(f"[{drone_name}] Landed successfully.")
                    self.get_logger().info(f"[{drone_name}] Shutting down connection.")
                    drone.end()
                    return True
    
                except Exception as e:
                    self.get_logger().warn(f"[{drone_name}] Land attempt {i+1} failed: {e}")
                    time.sleep(1.5 * (i + 1))  # backoff
    
            self.get_logger().error(f"[{drone_name}] FAILED to land after {max_retries} attempts.")
            return False
    
        except Exception as e:
            self.get_logger().error(f"[{drone_name}] Exception in safe_land: {e}")
            return False

    def on_shutdown(self):
        self.get_logger().info('Shutting down node, landing all drones...')
        self.land_all_drones()
        time.sleep(10)  # Ensure there is enough time to land all drones
        self.get_logger().info('All drones landed and sockets closed.')

    def shutdown_listener(self):
        while rclpy.ok():
            time.sleep(1)  # Keep the thread alive

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

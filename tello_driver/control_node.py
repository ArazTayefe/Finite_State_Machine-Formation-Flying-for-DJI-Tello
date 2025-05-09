import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from mocap4r2_msgs.msg import RigidBodies
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import threading
import sys, select, termios, tty

class DisplacementGeneratorNode(Node):
    def __init__(self):
        super().__init__('displacement_generator_node')
        self.drone_publishers = {}
        # DJI drone IDs (as strings) and the Leader_5G_Drone is assumed to have rigid body id '5'
        self.drones = {'Alpha': '1', 'Bravo': '2', 'Charlie': '3'}
        self.rigid_body_data = {}
        self.orientation_data = {}
        self.marker_publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.landing_publishers = {}
        self.phase_publisher = self.create_publisher(Int32, 'mission_phase', 10)


        for drone in self.drones:
            topic_name = f'{drone}_land'
            self.landing_publishers[drone] = self.create_publisher(Bool, topic_name, 10)
            for axis in ['Vel_x', 'Vel_y', 'Vel_z', 'AngVel_yaw']:
                topic_name = f'{drone}_cmd_{axis}'
                self.drone_publishers[(drone, axis)] = self.create_publisher(Float64, topic_name, 10)

        self.create_subscription(RigidBodies, '/rigid_bodies', self.handle_rigid_bodies, 10)
        self.broadcast_static_world_frame()

        # Control gains
        self.K_p_z = 3.0
        self.K_d_z = 1.5
        self.K_p_x = 3.0
        self.K_d_x = 1.5
        self.K_p_y = 3.0
        self.K_d_y = 1.5
        self.K_p_yaw = 5.0
        self.K_d_yaw = 2.0

        # Remove hard-coded initial positions; capture them from the mocap data
        self.initial_positions = {}      # e.g., {'Alpha': np.array([...]), ... , 'Leader_5G_Drone': np.array([...])}
        self.initial_orientations = {}   # similar for orientations

        # Desired states (will be updated per phase)
        self.desired_positions = {}
        self.desired_velocities = {drone: np.array([0.0, 0.0, 0.0]) for drone in self.drones}
        self.desired_yaws = {drone: 0.0 for drone in self.drones}
        self.desired_yaw_rates = {drone: 0.0 for drone in self.drones}

        self.last_positions = {drone: None for drone in self.drones}
        self.last_yaws = {drone: None for drone in self.drones}
        self.last_yaw_rates = {drone: None for drone in self.drones}
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        # State machine variables for the maneuver
        self.phase = 0  # starting phase (e.g., waiting for Leader_5G_Drone altitude)
        self.phase_start_time = self.get_clock().now().nanoseconds / 1e9

        # Formation parameters (adjust as needed)
        self.triangle_radius = 0.75 
        self.rotation_rate = np.pi / 10.0  # rad/sec for rotating formation

        self.create_timer(0.03, self.update)
        keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        keyboard_thread.start()
        rclpy.get_default_context().on_shutdown(self.on_shutdown)

    def broadcast_static_world_frame(self):
        broadcaster = StaticTransformBroadcaster(self)
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'world'
        static_transformStamped.child_frame_id = 'map'
        static_transformStamped.transform.rotation.w = 1.0
        broadcaster.sendTransform(static_transformStamped)

    def handle_rigid_bodies(self, msg):
        self.rigid_body_msg = msg  # Save incoming message for processing

    def update(self):
        if not hasattr(self, 'rigid_body_msg'):
            return

        current_time_ros = self.get_clock().now()
        current_time = current_time_ros.nanoseconds / 1e9
        dt = (current_time - self.last_time)
        self.last_time = current_time

        Leader_5G_Drone_position = None
        Leader_5G_Drone_orientation = None

        # Process each rigid body from mocap data.
        for rigid_body in self.rigid_body_msg.rigidbodies:
            if rigid_body.rigid_body_name == '5':  # Leader_5G_Drone's rigid body id
                Leader_5G_Drone_position = np.array([rigid_body.pose.position.x,
                                         rigid_body.pose.position.y,
                                         rigid_body.pose.position.z])
                Leader_5G_Drone_orientation = rigid_body.pose.orientation
                self.rigid_body_data['Leader_5G_Drone'] = Leader_5G_Drone_position
                self.orientation_data['Leader_5G_Drone'] = Leader_5G_Drone_orientation
                # Capture initial Leader_5G_Drone position/orientation if not already done.
                if 'Leader_5G_Drone' not in self.initial_positions:
                    self.initial_positions['Leader_5G_Drone'] = Leader_5G_Drone_position
                    self.initial_orientations['Leader_5G_Drone'] = Leader_5G_Drone_orientation

            drone = next((name for name, id in self.drones.items() if id == rigid_body.rigid_body_name), None)
            if drone:
                current_position = np.array([rigid_body.pose.position.x,
                                             rigid_body.pose.position.y,
                                             rigid_body.pose.position.z])
                self.rigid_body_data[drone] = current_position
                self.orientation_data[drone] = rigid_body.pose.orientation

                # Capture initial position/orientation for each drone once.
                if drone not in self.initial_positions:
                    self.initial_positions[drone] = current_position
                    self.initial_orientations[drone] = rigid_body.pose.orientation
                    # Also, set desired positions to start at these captured values.
                    self.desired_positions[drone] = current_position.copy()

                if self.last_positions[drone] is not None:
                    current_velocity = (current_position - self.last_positions[drone]) / dt
                    self.control_drone(drone, current_position, current_velocity, dt)
                self.last_positions[drone] = current_position

                current_yaw = self.get_yaw_from_quaternion(self.orientation_data[drone])
                if self.last_yaws[drone] is not None:
                    current_yaw_rate = (current_yaw - self.last_yaws[drone]) / dt
                    self.control_yaw(drone, current_yaw, current_yaw_rate)
                self.last_yaws[drone] = current_yaw

        # Finite-State-Machine (FSM) Process
        # Only proceed with FSM if Leader_5G_Drone data is available.
        if Leader_5G_Drone_position is not None and Leader_5G_Drone_orientation is not None:
            Leader_5G_Drone_alt = Leader_5G_Drone_position[2]

            if self.phase == 0:
                # Phase 0: Wait until the Leader_5G_Drone is stable at 0.5 m for at least 2 seconds
                altitude_error = abs(Leader_5G_Drone_alt - 0.5)

                if altitude_error < 0.1:
                    if not hasattr(self, 'phase0_stable_start_time'):
                        self.phase0_stable_start_time = current_time
                    elif current_time - self.phase0_stable_start_time > 2.0:  # 2 seconds stable
                        self.get_logger().info("Phase 0 complete: Leader_5G_Drone stable at 0.5 m. Starting phase 1 (Alpha).")
                        self.phase = 1
                        self.phase_start_time = current_time
                        del self.phase0_stable_start_time  # clean up
                else:
                    # Reset if it strays too far
                    if hasattr(self, 'phase0_stable_start_time'):
                        del self.phase0_stable_start_time

            elif self.phase == 1:
                # Phase 1: Move from initial positions directly to triangle formation and following the leader drone
                t_phase = current_time - self.phase_start_time
                duration = 5.0
                ratio = np.clip(t_phase / duration, 0.0, 1.0)
                yaw = self.get_yaw_from_quaternion(Leader_5G_Drone_orientation)

                offsets = {'Alpha': 0.0,
                            'Bravo': 3 * np.pi / 4,     # 135°
                            'Charlie': -3 * np.pi / 4   # -135°
                        }
                altitude_offsets = {'Alpha': 0.45, 'Bravo': 0.6, 'Charlie': 0.75}

                for drone in self.drones:
                    final_offset = np.array([
                        self.triangle_radius * np.cos((yaw + offsets[drone])),
                        self.triangle_radius * np.sin((yaw + offsets[drone]))
                    ])
                    final_pos_xy = np.array([Leader_5G_Drone_position[0], Leader_5G_Drone_position[1]]) + final_offset
                    start_xy = self.initial_positions[drone][:2]  # <- start from where they actually are
                    interp_xy = start_xy + ratio * (final_pos_xy - start_xy)

                    target_z = Leader_5G_Drone_position[2] + altitude_offsets[drone]
                    self.desired_positions[drone] = np.array([interp_xy[0], interp_xy[1], target_z])
                    self.desired_yaws[drone] = (yaw + offsets[drone]) - np.pi/2

                if Leader_5G_Drone_alt > 0.7:
                    self.get_logger().info("Leader_5G_Drone altitude changed: starting rotating formation (phase 2).")
                    self.phase = 2
                    self.phase_start_time = current_time

            elif self.phase == 2:
                # Phase 2: Continuous rotating formation around the Leader_5G_Drone.
                t_phase = current_time - self.phase_start_time
                yaw = self.get_yaw_from_quaternion(Leader_5G_Drone_orientation) - np.pi/2
                base_angle = self.rotation_rate * t_phase
                phase_offsets = {'Alpha': 0.0,
                            'Bravo': 3 * np.pi / 4,     # 135°
                            'Charlie': -3 * np.pi / 4   # -135°
                        }
                altitude_offsets = {'Alpha': 0.45, 'Bravo': 0.6, 'Charlie': 0.75}
                for drone in self.drones:
                    angle = base_angle + phase_offsets[drone]
                    offset_xy = np.array([self.triangle_radius * np.cos(angle),
                                          self.triangle_radius * np.sin(angle)])
                    current_alt = Leader_5G_Drone_position[2] + altitude_offsets[drone]
                    self.desired_positions[drone] = np.array([Leader_5G_Drone_position[0] + offset_xy[0],
                                                               Leader_5G_Drone_position[1] + offset_xy[1],
                                                               current_alt])
                    self.desired_yaws[drone] = angle + np.pi  # drone's camera faces outward
                if Leader_5G_Drone_alt < 0.6:
                    self.get_logger().info("Leader_5G_Drone altitude dropped: starting return to base (phase 3).")
                    self.phase = 3
                    self.phase_start_time = current_time

            elif self.phase == 3:
                if not hasattr(self, 'landing_index'):
                    self.landing_order = ['Alpha', 'Bravo', 'Charlie']
                    self.landing_index = 0
                    self.phase_start_time = current_time
                    self.landing_subphase = 0  # 0 = land, 1 = wait
                    self.get_logger().info("Phase 3 initiated: Sequential landing starting.")

                drone = self.landing_order[self.landing_index]

                if self.landing_subphase == 0:
                    self.get_logger().info(f"{drone} initiating landing at current position.")
                    if drone in self.landing_publishers:
                        msg = Bool()
                        msg.data = True
                        self.landing_publishers[drone].publish(msg)
                        # Send rc 0 0 0 0 to keep drone responsive
                        zero_msg = Float64()
                        zero_msg.data = 0.0
                        for axis in ['Vel_x', 'Vel_y', 'Vel_z', 'AngVel_yaw']:
                            self.drone_publishers[(drone, axis)].publish(zero_msg)
                    else:
                        self.get_logger().warn(f"No landing publisher for {drone}.")
                    self.landing_subphase = 1
                    self.phase_start_time = current_time

                elif self.landing_subphase == 1:
                    if current_time - self.phase_start_time > 5.0:
                        self.landing_index += 1
                        self.landing_subphase = 0
                        self.phase_start_time = current_time

                        if self.landing_index >= len(self.landing_order):
                            self.get_logger().info("Phase 3 complete: All drones sent landing command.")
                            del self.landing_index
                            del self.landing_order
                            del self.landing_subphase
                            self.phase += 1  # or stop here


            # Back to Mission Control Center
            elif self.phase == 4:
                if not hasattr(self, 'landing_index'):
                    self.landing_order = ['Alpha', 'Bravo', 'Charlie']
                    self.landing_index = 0
                    self.phase_start_time = current_time
                    self.landing_subphase = 0  # 0 = go to XY, 1 = land
                    self.get_logger().info("Phase 3 initiated: Sequential landing starting.")
            
                drone = self.landing_order[self.landing_index]
                target_xy = self.initial_positions[drone][:2]
                current_pos = self.rigid_body_data[drone]
                current_z = current_pos[2]
            
                if self.landing_subphase == 0:
                    # Step 1: Go to initial X, Y and maintain current Z
                    desired_pos = np.array([target_xy[0], target_xy[1], current_z])
                    self.desired_positions[drone] = desired_pos
                    self.desired_yaws[drone] = 0.0
            
                    xy_error = np.linalg.norm(current_pos[:2] - target_xy)
                    if xy_error < 0.1:
                        self.get_logger().info(f"{drone} reached XY target. Sending land command.")
                        self.send_landing_command(drone)
                        self.landing_subphase = 1
                        self.phase_start_time = current_time
            
                elif self.landing_subphase == 1:
                    # Step 2: Wait for a few seconds to allow landing
                    if current_time - self.phase_start_time > 5.0:
                        self.landing_index += 1
                        self.landing_subphase = 0
                        self.phase_start_time = current_time
            
                        if self.landing_index >= len(self.landing_order):
                            self.get_logger().info("Phase 3 complete: All drones sent landing command.")
                            del self.landing_index
                            del self.landing_order
                            del self.landing_subphase
                            self.phase += 1  # or stop

        self.publish_mission_phase()

        # End of state machine update

        self.update_markers()

    def publish_mission_phase(self):
        msg = Int32()
        msg.data = self.phase
        self.phase_publisher.publish(msg)

    def keyboard_listener(self):
        """Non-blocking keyboard listener. Press 's' to send stop commands."""
        self.get_logger().info("Keyboard listener started. Press 's' to send stop commands.")
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    if key == 's':
                        self.get_logger().info("Stop key 's' pressed, sending stop commands.")
                        self.send_stop_commands()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def send_stop_commands(self):
        """Send a stop command to all drones."""
        for drone in self.drones:
            stop_msg = Float64()
            stop_msg.data = 0.0
            for axis in ['Vel_x', 'Vel_y', 'Vel_z', 'AngVel_yaw']:
                self.drone_publishers[(drone, axis)].publish(stop_msg)
            # Optionally, send a landing command as well:
            if drone in self.landing_publishers:
                landing_msg = Bool()
                landing_msg.data = True
                self.landing_publishers[drone].publish(landing_msg)

    def get_yaw_from_quaternion(self, q):
        quaternion = [q.x, q.y, q.z, q.w]
        norm = np.linalg.norm(quaternion)
        if norm != 0:
            quaternion = [elem / norm for elem in quaternion]
        q_x, q_y, q_z, q_w = quaternion
        yaw = np.arctan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2)) - np.pi/2
        return yaw

    def control_drone(self, drone, current_position, current_velocity, dt):
        desired_position = self.desired_positions[drone]
        desired_velocity = self.desired_velocities[drone]
        error_position = desired_position - current_position
        error_velocity = desired_velocity - current_velocity

        commanded_velocity_x = self.K_p_x * error_position[0] + self.K_d_x * error_velocity[0]
        commanded_velocity_y = self.K_p_y * error_position[1] + self.K_d_y * error_velocity[1]
        commanded_velocity_z = self.K_p_z * error_position[2] + self.K_d_z * error_velocity[2]

        # Simple saturation (adjust limits as needed)
        if current_position[0] > 1.7 or current_position[0] < -1.7:
            commanded_velocity_x = 0.0
        if current_position[1] > 2.7 or current_position[1] < -2.7:
            commanded_velocity_y = 0.0
        if current_position[2] > 1.8:
            commanded_velocity_z = 0.0

        commanded_velocity = np.array([commanded_velocity_x, commanded_velocity_y, commanded_velocity_z])
        current_yaw = self.get_yaw_from_quaternion(self.orientation_data[drone])
        R = np.array([
            [np.cos(current_yaw), np.sin(current_yaw)],
            [-np.sin(current_yaw), np.cos(current_yaw)]
        ])
        commanded_velocity_drone_frame = np.dot(R, commanded_velocity[:2])
        commanded_velocity_drone_frame = np.append(commanded_velocity_drone_frame, commanded_velocity[2])
        self.publish_velocity(drone, commanded_velocity_drone_frame)

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def control_yaw(self, drone, current_yaw, current_yaw_rate):
        desired_yaw = self.desired_yaws[drone]
        error_yaw = self.normalize_angle(desired_yaw - current_yaw + np.pi)
        if abs(error_yaw) < np.deg2rad(5):
            commanded_yaw_rate = 0.0
        else:
            desired_yaw_rate = self.desired_yaw_rates[drone]
            error_yaw_rate = desired_yaw_rate - current_yaw_rate
            commanded_yaw_rate = self.K_p_yaw * error_yaw + self.K_d_yaw * error_yaw_rate
            max_yaw_rate = 0.5
            commanded_yaw_rate = max(min(commanded_yaw_rate, max_yaw_rate), -max_yaw_rate)
            alpha = 0.5
            if self.last_yaw_rates[drone] is not None:
                commanded_yaw_rate = alpha * commanded_yaw_rate + (1 - alpha) * self.last_yaw_rates[drone]
            self.last_yaw_rates[drone] = commanded_yaw_rate
        self.publish_yaw_rate(drone, commanded_yaw_rate)

    def publish_velocity(self, drone, commanded_velocity):
        msg_x = Float64()
        msg_x.data = commanded_velocity[0]
        self.drone_publishers[(drone, 'Vel_x')].publish(msg_x)

        msg_y = Float64()
        msg_y.data = commanded_velocity[1]
        self.drone_publishers[(drone, 'Vel_y')].publish(msg_y)

        msg_z = Float64()
        msg_z.data = commanded_velocity[2]
        self.drone_publishers[(drone, 'Vel_z')].publish(msg_z)

    def publish_yaw_rate(self, drone, commanded_yaw_rate):
        msg_yaw = Float64()
        msg_yaw.data = float(commanded_yaw_rate)
        self.drone_publishers[(drone, 'AngVel_yaw')].publish(msg_yaw)

    def send_landing_command(self, drone):
        landing_msg = Bool()
        landing_msg.data = True
        self.landing_publishers[drone].publish(landing_msg)

    def update_markers(self):
        marker_array = MarkerArray()
        marker_id = 0

        for drone, position in self.rigid_body_data.items():
            if position is not None:
                cube_marker = Marker()
                cube_marker.header.frame_id = "map"
                cube_marker.type = Marker.CUBE
                cube_marker.action = Marker.ADD
                cube_marker.pose.position.x = float(position[0])
                cube_marker.pose.position.y = float(position[1])
                cube_marker.pose.position.z = float(position[2])
                cube_marker.scale.x = 0.1
                cube_marker.scale.y = 0.1
                cube_marker.scale.z = 0.1
                cube_marker.color.a = 1.0
                cube_marker.color.r = 1.0
                cube_marker.color.g = 0.0
                cube_marker.color.b = 0.0
                cube_marker.id = marker_id
                marker_array.markers.append(cube_marker)
                marker_id += 1

                arrow_marker = Marker()
                arrow_marker.header.frame_id = "map"
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                arrow_marker.pose.position = cube_marker.pose.position
                arrow_marker.pose.orientation = self.orientation_data[drone]
                arrow_marker.scale.x = 0.2
                arrow_marker.scale.y = 0.05
                arrow_marker.scale.z = 0.05
                arrow_marker.color.a = 1.0
                arrow_marker.color.r = 0.0
                arrow_marker.color.g = 1.0
                arrow_marker.color.b = 0.0
                arrow_marker.id = marker_id
                marker_array.markers.append(arrow_marker)
                marker_id += 1

        if 'Leader_5G_Drone' in self.rigid_body_data and self.rigid_body_data['Leader_5G_Drone'] is not None:
            Leader_5G_Drone_position = self.rigid_body_data['Leader_5G_Drone']
            Leader_5G_Drone_orientation = self.orientation_data['Leader_5G_Drone']
            Leader_5G_Drone_marker = Marker()
            Leader_5G_Drone_marker.header.frame_id = "map"
            Leader_5G_Drone_marker.type = Marker.CUBE
            Leader_5G_Drone_marker.action = Marker.ADD
            Leader_5G_Drone_marker.pose.position.x = float(Leader_5G_Drone_position[0])
            Leader_5G_Drone_marker.pose.position.y = float(Leader_5G_Drone_position[1])
            Leader_5G_Drone_marker.pose.position.z = float(Leader_5G_Drone_position[2])
            Leader_5G_Drone_marker.pose.orientation = Leader_5G_Drone_orientation
            Leader_5G_Drone_marker.scale.x = 0.2
            Leader_5G_Drone_marker.scale.y = 0.2
            Leader_5G_Drone_marker.scale.z = 0.2
            Leader_5G_Drone_marker.color.a = 1.0
            Leader_5G_Drone_marker.color.r = 0.0
            Leader_5G_Drone_marker.color.g = 0.0
            Leader_5G_Drone_marker.color.b = 1.0
            Leader_5G_Drone_marker.id = marker_id
            marker_array.markers.append(Leader_5G_Drone_marker)
            marker_id += 1
        else:
            self.get_logger().warn("Leader_5G_Drone data not available or invalid.")

        self.marker_publisher.publish(marker_array)

    def on_shutdown(self):
        self.get_logger().info("Shutdown initiated: Sending stop and landing commands repeatedly.")
        shutdown_duration = 1.0  # seconds
        shutdown_interval = 0.1  # seconds
        iterations = int(shutdown_duration / shutdown_interval)
        for i in range(iterations):
            self.send_stop_commands()
            rclpy.spin_once(self, timeout_sec=shutdown_interval)
        self.get_logger().info("Shutdown complete: Commands sent.")

def main(args=None):
    rclpy.init(args=args)
    node = DisplacementGeneratorNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

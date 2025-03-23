import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from mocap4r2_msgs.msg import RigidBodies
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class DisplacementGeneratorNode(Node):
    def __init__(self):
        super().__init__('displacement_generator_node')
        self.drone_publishers = {}
        self.drones = {'Alpha': '1', 'Bravo': '2', 'Charlie': '3'}
        self.rigid_body_data = {}
        self.orientation_data = {}
        self.marker_publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        for drone in self.drones:
            for axis in ['Vel_x', 'Vel_y', 'Vel_z', 'AngVel_yaw']:
                topic_name = f'{drone}_cmd_{axis}'
                self.drone_publishers[(drone, axis)] = self.create_publisher(Float64, topic_name, 10)

        self.create_subscription(RigidBodies, '/rigid_bodies', self.handle_rigid_bodies, 10)
        self.broadcast_static_world_frame()

        self.K_p_z = 3.0  # Proportional gain
        self.K_d_z = 1.5  # Derivative gain

        self.K_p_x = 3.0  # Proportional gain
        self.K_d_x = 1.5  # Derivative gain

        self.K_p_y = 3.0  # Proportional gain
        self.K_d_y = 1.5  # Derivative gain

        self.K_p_yaw = 5.0  # Proportional gain for yaw
        self.K_d_yaw = 2.0  # Integral gain for yaw

        self.desired_positions = {
            'Alpha': np.array([0.0, 0.5, 1.0]),
            'Bravo': np.array([-0.5, 0.5, 1.0]),
            'Charlie': np.array([0.5, 0.5, 1.0])
        }
        self.desired_velocities = {
            'Alpha': np.array([0.0, 0.0, 0.0]),
            'Bravo': np.array([0.0, 0.0, 0.0]),
            'Charlie': np.array([0.0, 0.0, 0.0])
        }
        self.desired_yaws = {
            'Alpha': 0.0,
            'Bravo': 0.0,
            'Charlie': 0.0
        }
        self.desired_yaw_rates = {
            'Alpha': 0.0,
            'Bravo': 0.0,
            'Charlie': 0.0
        }

        self.last_positions = {drone: None for drone in self.drones}
        self.last_yaws = {drone: None for drone in self.drones}
        self.last_yaw_rates = {drone: None for drone in self.drones}
        self.last_time = self.get_clock().now()

        self.create_timer(0.03, self.update)  # Call update function every 0.01 seconds
        rclpy.get_default_context().on_shutdown(self.on_shutdown)  # Register the shutdown callback

    def broadcast_static_world_frame(self):
        broadcaster = StaticTransformBroadcaster(self)
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'world'
        static_transformStamped.child_frame_id = 'map'
        static_transformStamped.transform.rotation.w = 1.0
        broadcaster.sendTransform(static_transformStamped)

    def handle_rigid_bodies(self, msg):
        self.rigid_body_msg = msg  # Store the message for processing in the update function

    def update(self):
        if not hasattr(self, 'rigid_body_msg'):
            return  # If the message hasn't been received yet, return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        ugv_position = None
        ugv_orientation = None

        for rigid_body in self.rigid_body_msg.rigidbodies:
            # self.get_logger().info(f"Processing rigid body: {rigid_body.rigid_body_name}")
            if rigid_body.rigid_body_name == '5':  # UGV rigid body ID
                # self.get_logger().info(f"UGV position: {rigid_body.pose.position}")
                # self.get_logger().info(f"UGV orientation: {rigid_body.pose.orientation}")
                ugv_position = np.array([rigid_body.pose.position.x, rigid_body.pose.position.y, rigid_body.pose.position.z])
                ugv_orientation = rigid_body.pose.orientation
                self.rigid_body_data['UGV'] = ugv_position
                self.orientation_data['UGV'] = ugv_orientation

            drone = next((name for name, id in self.drones.items() if id == rigid_body.rigid_body_name), None)
            if drone:
                current_position = np.array([rigid_body.pose.position.x, rigid_body.pose.position.y, rigid_body.pose.position.z])
                self.rigid_body_data[drone] = current_position
                self.orientation_data[drone] = rigid_body.pose.orientation

                if self.last_positions[drone] is not None:
                    current_velocity = (current_position - self.last_positions[drone]) / dt
                    self.control_drone(drone, current_position, current_velocity, dt)

                self.last_positions[drone] = current_position

                current_yaw = self.get_yaw_from_quaternion(self.orientation_data[drone])
                if self.last_yaws[drone] is not None:
                    current_yaw_rate = (current_yaw - self.last_yaws[drone]) / dt
                    self.control_yaw(drone, current_yaw, current_yaw_rate)

                self.last_yaws[drone] = current_yaw

        if ugv_position is not None and ugv_orientation is not None:
            for drone in self.drones:
                self.update_desired_position(drone, ugv_position, ugv_orientation)

        self.update_markers()

    def get_yaw_from_quaternion(self, q):
        quaternion = [q.x, q.y, q.z, q.w]
        norm = np.linalg.norm(quaternion)
        if norm != 0:
            quaternion = [q / norm for q in quaternion]
        q_x, q_y, q_z, q_w = quaternion
        yaw = np.arctan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2)) - np.pi/2
        return yaw

    def control_drone(self, drone, current_position, current_velocity, dt):
        desired_position = self.desired_positions[drone]
        desired_velocity = self.desired_velocities[drone]

        error_position = desired_position - current_position
        error_velocity = desired_velocity - current_velocity

        # Separate calculations for commanded velocities
        commanded_velocity_x = self.K_p_x * error_position[0] + self.K_d_x * error_velocity[0]
        commanded_velocity_y = self.K_p_y * error_position[1] + self.K_d_y * error_velocity[1]
        commanded_velocity_z = self.K_p_z * error_position[2] + self.K_d_z * error_velocity[2]

        if current_position[0] > 1.7 or current_position[0] < -1.7:
            commanded_velocity_x = 0.0
        if current_position[1] > 2.7 or current_position[1] < -2.7:
            commanded_velocity_y = 0.0
        if current_position[2] > 1.8:
            commanded_velocity_z = 0.0

        commanded_velocity = np.array([commanded_velocity_x, commanded_velocity_y, commanded_velocity_z])

        # Get current yaw angle
        current_yaw = self.get_yaw_from_quaternion(self.orientation_data[drone])

        # Calculate the rotation matrix
        R = np.array([
            [np.cos(current_yaw), np.sin(current_yaw)],
            [-np.sin(current_yaw), np.cos(current_yaw)]
        ])

        # The commanded velocity in the drone's coordinate frame
        commanded_velocity_drone_frame = np.dot(R, commanded_velocity[:2])
        commanded_velocity_drone_frame = np.append(commanded_velocity_drone_frame, commanded_velocity[2])

        self.publish_velocity(drone, commanded_velocity_drone_frame)

    def normalize_angle(self, angle):
        """
        Normalize an angle to the range [-pi, pi].
        """
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def control_yaw(self, drone, current_yaw, current_yaw_rate):
        desired_yaw = self.desired_yaws[drone]  # Get desired yaw for the drone
        error_yaw = self.normalize_angle(desired_yaw - current_yaw + np.pi)  # Calculate yaw error

        if abs(error_yaw) < np.deg2rad(5):  # If yaw error is less than 5 degrees, set yaw rate command to zero
            commanded_yaw_rate = 0.0
        else:
            desired_yaw_rate = self.desired_yaw_rates[drone]  # Fetch the desired yaw rate from the dictionary
            error_yaw_rate = desired_yaw_rate - current_yaw_rate  # Calculate yaw rate error

            # Calculate commanded yaw rate using PD control
            commanded_yaw_rate = self.K_p_yaw * error_yaw + self.K_d_yaw * error_yaw_rate

            # Implement saturation limits
            max_yaw_rate = 0.5  # Example limit, adjust as needed
            commanded_yaw_rate = max(min(commanded_yaw_rate, max_yaw_rate), -max_yaw_rate)

            alpha = 0.5  # Smoothing factor, adjust as needed
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

    def update_desired_position(self, drone, ugv_position, ugv_orientation):
        if drone == 'Alpha':
            offset = np.array([0.0, 0.4, 0.0])  # Offset for Alpha in X, Y (Z is set separately)
            z_offset = 0.50  # Z=0.5 for Alpha
        elif drone == 'Bravo':
            offset = np.array([-0.4, -0.4, 0.0])  # Offset for Bravo in X, Y (Z is set separately)
            z_offset = 0.75  # Z=1.0 for Bravo
        elif drone == 'Charlie':
            offset = np.array([0.4, -0.4, 0.0])  # Offset for Charlie in X, Y (Z is set separately)
            z_offset = 1.00  # Z=1.5 for Charlie
        else:
            offset = np.array([0.0, 0.0, 0.0])
            z_offset = 0.0

        yaw = self.get_yaw_from_quaternion(ugv_orientation)
        rotation_matrix = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw), np.cos(yaw)]
        ])

        rotated_offset = np.dot(rotation_matrix, offset[:2])
        desired_position = ugv_position[:2] + rotated_offset
        self.desired_positions[drone] = np.array([desired_position[0], desired_position[1], z_offset + ugv_position[2]])
        self.desired_yaws[drone] = yaw

    def update_markers(self):
        marker_array = MarkerArray()
        marker_id = 0

        # Create markers for drones
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
                cube_marker.scale.z = 0.1  # Size of the cube
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

        # Create marker for UGV
        if 'UGV' in self.rigid_body_data and self.rigid_body_data['UGV'] is not None:
            ugv_position = self.rigid_body_data['UGV']
            ugv_orientation = self.orientation_data['UGV']

            ugv_marker = Marker()
            ugv_marker.header.frame_id = "map"
            ugv_marker.type = Marker.CUBE
            ugv_marker.action = Marker.ADD
            ugv_marker.pose.position.x = float(ugv_position[0])
            ugv_marker.pose.position.y = float(ugv_position[1])
            ugv_marker.pose.position.z = float(ugv_position[2])
            ugv_marker.pose.orientation = ugv_orientation
            ugv_marker.scale.x = 0.2
            ugv_marker.scale.y = 0.2
            ugv_marker.scale.z = 0.2  # Size of the UGV cube
            ugv_marker.color.a = 1.0
            ugv_marker.color.r = 0.0
            ugv_marker.color.g = 0.0
            ugv_marker.color.b = 1.0
            ugv_marker.id = marker_id
            marker_array.markers.append(ugv_marker)
            marker_id += 1
        else:
            self.get_logger().warn("UGV data not available or invalid.")

        self.marker_publisher.publish(marker_array)

    def on_shutdown(self):
        # self.get_logger().info('Shutting down node, stopping drones...')
        for drone in self.drones:
            stop_msg = Float64()
            stop_msg.data = 0.0
            for axis in ['Vel_x', 'Vel_y', 'Vel_z', 'AngVel_yaw']:
                self.drone_publishers[(drone, axis)].publish(stop_msg)
                # self.get_logger().info(f'Stopped {drone} cmd_{axis}')
        # self.get_logger().info('All drones stopped.')

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

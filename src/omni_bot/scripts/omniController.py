#!/usr/bin/env python3

import rclpy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf_transformations import euler_from_quaternion  # Add this import

class OdometryPublisher:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_radius = 0.042
        self.expected_distance = 0.0
        self.actual_distance = 0.0

    def update(self, linear_velocity_x, linear_velocity_y, angular_velocity, delta_t):
        delta_theta = angular_velocity * delta_t
        avg_theta = self.theta + delta_theta 

        delta_x = (linear_velocity_x * math.cos(avg_theta) - linear_velocity_y * math.sin(avg_theta)) * delta_t
        delta_y = (linear_velocity_x * math.sin(avg_theta) + linear_velocity_y * math.cos(avg_theta)) * delta_t

        self.expected_distance += math.sqrt(linear_velocity_x ** 2 + linear_velocity_y ** 2) * delta_t
        self.actual_distance += self.wheel_radius * abs(angular_velocity) * delta_t

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

    def get_odometry(self):
        return self.x, self.y, self.theta

class OmniDriveController(Node):
    def __init__(self):
        super().__init__('omni_drive_controller')
        self.tf_broadcaster = TransformBroadcaster()
        self.map_to_odom_broadcaster = TransformBroadcaster()
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        self.odometry_publisher = OdometryPublisher()
        self.last_callback_time = self.get_clock().now()
        
        self.twist_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.twist_callback, 10)
        
        self.odom_publisher = self.create_publisher(
            Odometry, 'odom_test', 1)
        
        self.twist_publisher = self.create_publisher(
            Twist, 'cmd_vel', 1)
        
        self.rate = self.create_rate(50)  # 50 Hz update rate
    
    def twist_callback(self, twist_msg):
        current_time = self.get_clock().now()
        delta_t = (current_time - self.last_callback_time).to_sec()
        Vx = twist_msg.linear.x
        Vy = twist_msg.linear.y
        Vw = twist_msg.angular.z * self.wheel_radius
        
        self.odometry_publisher.update(Vx, Vy, Vw, delta_t)
        self.robot_x, self.robot_y, self.robot_theta = self.odometry_publisher.get_odometry()

        # Publish odometry position
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.robot_x
        odom_msg.pose.pose.position.y = self.robot_y
        quaternion = quaternion_from_euler(0.0, 0.0, self.robot_theta)
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]
        odom_msg.twist.twist.linear.x = Vx
        odom_msg.twist.twist.linear.y = Vy
        odom_msg.twist.twist.angular.z = Vw

        self.odom_publisher.publish(odom_msg)
        
        # Publish tf transform
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = current_time.to_msg()
        transform_stamped.header.frame_id = 'odom'
        transform_stamped.child_frame_id = 'base_link'
        transform_stamped.transform.translation.x = self.robot_x
        transform_stamped.transform.translation.y = self.robot_y
        transform_stamped.transform.rotation.x = quaternion[0]
        transform_stamped.transform.rotation.y = quaternion[1]
        transform_stamped.transform.rotation.z = quaternion[2]
        transform_stamped.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(transform_stamped)

        # Publish map to odom transform
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = current_time.to_msg()
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = 'odom'
        map_to_odom.transform.translation.x = self.robot_x
        map_to_odom.transform.translation.y = self.robot_y
        map_to_odom.transform.rotation.x = 0.0
        map_to_odom.transform.rotation.y = 0.0
        map_to_odom.transform.rotation.z = 0.0
        map_to_odom.transform.rotation.w = 1.0
        self.map_to_odom_broadcaster.sendTransform(map_to_odom)
        
        self.last_callback_time = current_time

def main(args=None):
    rclpy.init(args=args)
    omni_drive_controller = OmniDriveController()
    try:
        rclpy.spin(omni_drive_controller)
    finally:
        omni_drive_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

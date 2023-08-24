#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32
import tf2_ros
from tf2_geometry_msgs import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener
from tf2_ros.buffer import Buffer
from tf_transformations import euler_from_quaternion  

import numpy as np

class RobotFootprintNode(Node):
    def __init__(self):
        super().__init__('robot_footprint')
        self.get_logger().info("---------------------Robot Footprint Node Initalized----------------------")

        self.robot_radius = 0.5  # Radius of the robot's circular footprint in meters

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            10
        )

        self.polygon_pub = self.create_publisher(
            PolygonStamped,
            'robot_footprint_polygon',
            1
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def odometry_callback(self, odom_msg):
        """!odometry_callback Subscribe to Robots Odometry to publish location as a footprint

        @param odom_msg: robot pose @type odom_msg: Odometry
        """
        self.robot_x = odom_msg.pose.pose.position.x
        self.robot_y = odom_msg.pose.pose.position.y

        quaternion = [
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        ]
        euler_angles = euler_from_quaternion(quaternion)
        self.robot_theta = euler_angles[2]

    def timer_callback(self):
        """!timer_callback Footprint publisher loop /robot_footprint_polygon
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(seconds=0.0), rclpy.time.Duration(seconds=1.0)
            )
        except tf2_ros.LookupException as e:
            self.get_logger().info("LookupException: %s" % str(e))
            return
        except tf2_ros.ConnectivityException as e:
            self.get_logger().info("ConnectivityException: %s" % str(e))
            return
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().info("ExtrapolationException: %s" % str(e))
            return

        self.robot_x = transform.transform.translation.x
        self.robot_y = transform.transform.translation.y

        euler_angles = euler_from_quaternion([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])
        self.robot_theta = euler_angles[2]

        polygon_msg = PolygonStamped()
        polygon_msg.header.stamp = self.get_clock().now().to_msg()
        polygon_msg.header.frame_id = 'map'
        polygon_msg.polygon.points = []

        num_points = 36
        for i in range(num_points):
            angle = i * (2 * np.pi / num_points)
            x = self.robot_x + self.robot_radius * np.cos(angle)
            y = self.robot_y + self.robot_radius * np.sin(angle)
            p=Point32()
            p.x=x
            p.y=y
            polygon_msg.polygon.points.append(p)

        self.polygon_pub.publish(polygon_msg)

def main(args=None):
    rclpy.init(args=args)
    robot_footprint_node = RobotFootprintNode()
    rclpy.spin(robot_footprint_node)
    robot_footprint_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

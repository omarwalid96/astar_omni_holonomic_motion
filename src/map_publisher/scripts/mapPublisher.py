#!/usr/bin/env python3

import rclpy
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from rclpy.node import Node


class MapPublisher(Node):

    def __init__(self):
        """!__init__ MapPublisher Node
        """
        super().__init__('map_publisher')
        self.get_logger().info("---------------------Map Node Initalized----------------------")
        self.occupancy_publisher = self.create_publisher(OccupancyGrid, 'map', 1)
        self.timer = self.create_timer(0.05,self.publishMap)
        # Map parameters
        self.resolution = 0.05  # meters per pixel
        self.map_size = 25.0    # meters
        self.free_square_size = 25.0  # meters

        # Calculate grid size
        self.grid_size = int(self.map_size / self.resolution)
        self.free_square_grid_size = int(self.free_square_size / self.resolution)

        # Create an empty map
        self.occupancy_map = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)

        # Fill in the free square
        self.start_idx = (self.grid_size - self.free_square_grid_size) // 2
        self.end_idx = self.start_idx + self.free_square_grid_size
        self.occupancy_map[self.start_idx:self.end_idx, self.start_idx:self.end_idx] = 0  # Free space

        for i in range(10): 
            # Add occupied borders
            self.occupancy_map[0+i, :] = 100  # Top border
            self.occupancy_map[-1-i, :] = 100  # Bottom border
            self.occupancy_map[:, 0+i] = 100  # Left border
            self.occupancy_map[:, -1-i] = 100  # Right border
        # Create TF broadcaster
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.rate = self.create_rate(10)

    def publishMap(self):
        """!publishMap Publish an occupancy grid map based on 0.05 resolution 
        """
        
        
        # Create OccupancyGrid message
        self.occupancy_msg = OccupancyGrid()
        self.occupancy_msg.header = Header()
        self.occupancy_msg.header.stamp = self.get_clock().now().to_msg()
        self.occupancy_msg.header.frame_id = 'map'
        self.occupancy_msg.info.map_load_time = self.get_clock().now().to_msg()
        self.occupancy_msg.info.resolution = self.resolution
        self.occupancy_msg.info.width = self.grid_size
        self.occupancy_msg.info.height = self.grid_size
        self.occupancy_msg.info.origin.position.x = 0.0
        self.occupancy_msg.info.origin.position.y = 0.0
        self.occupancy_msg.data = np.ravel(self.occupancy_map).tolist()

        self.tf_msg = TransformStamped()
        self.tf_msg.header = self.occupancy_msg.header
        self.tf_msg.child_frame_id = 'odom'
        self.tf_msg.transform.translation.x = 0.0
        self.tf_msg.transform.translation.y = 0.0
        self.tf_msg.transform.translation.z = 0.0
        self.tf_msg.transform.rotation.w = 1.0

        self.occupancy_publisher.publish(self.occupancy_msg)
        # self.tf_broadcaster.sendTransform(self.tf_msg)
        
def main(args=None):
    rclpy.init(args=args)
    map_publisher = MapPublisher()
    rclpy.spin(map_publisher)
    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
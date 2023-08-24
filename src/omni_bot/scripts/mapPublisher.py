#!/usr/bin/env python3

import rclpy
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

# def publish_occupancy_grid():
#     rclpy.init('map_publisher')

#     node = rclpy.create_node('map_publisher')
#     occupancy_publisher = node.create_publisher(OccupancyGrid, 'map', 1)

#     # Map parameters
#     resolution = 0.05  # meters per pixel
#     self. = 25.0    # meters
#     free_square_size = 25.0  # meters

#     # Calculate grid size
#     grid_size = int(map_size / resolution)
#     free_square_grid_size = int(free_square_size / resolution)

#     # Create an empty map
#     occupancy_map = np.zeros((grid_size, grid_size), dtype=np.int8)

#     # Fill in the free square
#     start_idx = (grid_size - free_square_grid_size) // 2
#     end_idx = start_idx + free_square_grid_size
#     occupancy_map[start_idx:end_idx, start_idx:end_idx] = 0  # Free space

#     for i in range(10): 
#         # Add occupied borders
#         occupancy_map[0+i, :] = 100  # Top border
#         occupancy_map[-1-i, :] = 100  # Bottom border
#         occupancy_map[:, 0+i] = 100  # Left border
#         occupancy_map[:, -1-i] = 100  # Right border

    

#     # Create TF broadcaster
#     tf_broadcaster = tf2_ros.TransformBroadcaster(node)

#     # Publish the OccupancyGrid and TF
#     rate = node.create_rate(10)  # Publish rate in Hz
#     while rclpy.ok():
#         print("Publish")
#         # Create OccupancyGrid message
#         occupancy_msg = OccupancyGrid()
#         occupancy_msg.header = Header()
#         occupancy_msg.header.stamp = node.get_clock().now().to_msg()
#         occupancy_msg.header.frame_id = 'map'
#         occupancy_msg.info.map_load_time = node.get_clock().now().to_msg()
#         occupancy_msg.info.resolution = resolution
#         occupancy_msg.info.width = grid_size
#         occupancy_msg.info.height = grid_size
#         occupancy_msg.info.origin.position.x = 0.0
#         occupancy_msg.info.origin.position.y = 0.0
#         occupancy_msg.data = np.ravel(occupancy_map).tolist()

#         tf_msg = TransformStamped()
#         tf_msg.header = occupancy_msg.header
#         tf_msg.child_frame_id = 'odom'
#         tf_msg.transform.translation.x = 0.0
#         tf_msg.transform.translation.y = 0.0
#         tf_msg.transform.translation.z = 0.0
#         tf_msg.transform.rotation.w = 1.0

#         occupancy_publisher.publish(occupancy_msg)
#         tf_broadcaster.sendTransform(tf_msg)
#         rate.sleep()

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     publish_occupancy_grid()


# import rclpy
# from rclpy.node import Node

# from std_msgs.msg import String
from rclpy.node import Node


class MapPublisher(Node):

    def __init__(self):
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
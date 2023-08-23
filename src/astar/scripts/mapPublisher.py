#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
import std_msgs.msg

def publish_occupancy_grid():
    

    # Map parameters
    resolution = 0.05  # meters per pixel
    map_size = 25.0    # meters
    free_square_size = 25.0  # meters

    # Calculate grid size
    grid_size = int(map_size / resolution)
    free_square_grid_size = int(free_square_size / resolution)

    # Create an empty map
    occupancy_map = np.zeros((grid_size, grid_size), dtype=np.int8)

    # Fill in the free square
    start_idx = (grid_size - free_square_grid_size) // 2
    end_idx = start_idx + free_square_grid_size
    occupancy_map[start_idx:end_idx, start_idx:end_idx] = 0  # Free space

    for i in range(10): 
        # Add occupied borders
        occupancy_map[0+i, :] = 100  # Top border
        occupancy_map[-1-i, :] = 100  # Bottom border
        occupancy_map[:, 0+i] = 100  # Left border
        occupancy_map[:, -1-i] = 100  # Right border

    # Create OccupancyGrid message
    occupancy_msg = nav_msgs.msg.OccupancyGrid()
    occupancy_msg.header.stamp = rospy.Time.now()
    occupancy_msg.header.frame_id = 'map'
    occupancy_msg.info.map_load_time = rospy.Time.now()
    occupancy_msg.info.resolution = resolution
    occupancy_msg.info.width = grid_size
    occupancy_msg.info.height = grid_size
    occupancy_msg.info.origin.position.x = 0.0
    occupancy_msg.info.origin.position.y = 0.0
    occupancy_msg.data = np.ravel(occupancy_map)

    # Create TF broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Publish the OccupancyGrid and TF
    while not rospy.is_shutdown():
        occupancy_msg.header.stamp = rospy.Time.now()
        tf_msg = geometry_msgs.msg.TransformStamped()
        tf_msg.header = occupancy_msg.header
        tf_msg.child_frame_id = 'odom'
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.w = 1.0

        occupancy_publisher.publish(occupancy_msg)
        tf_broadcaster.sendTransform(tf_msg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('map_publisher')

    try:
        
        rate = rospy.Rate(1)  # Publish rate in Hz
        occupancy_publisher = rospy.Publisher('map', nav_msgs.msg.OccupancyGrid, queue_size=1)
        publish_occupancy_grid()
    except rospy.ROSInterruptException:
        pass

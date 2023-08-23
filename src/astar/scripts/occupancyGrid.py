import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid

# Grid parameters
map_size = (25, 25)  # Map dimensions in meters
grid_resolution = 0.05  # Size of each cell in meters
grid_size = (int(map_size[0] / grid_resolution), int(map_size[1] / grid_resolution))  # Number of rows and columns

# Create an empty occupancy grid
occupancy_grid = np.zeros(grid_size, dtype=np.int8)  # Use int8 for the occupancy values

# Set the border width in cells
border_width = 1

# ROS Initialization
rospy.init_node('occupancy_grid_publisher')
pub = rospy.Publisher('occupancy_grid', OccupancyGrid, queue_size=1)
rate = rospy.Rate(1)  # Publish rate (1 Hz in this example)

# Simulate occupancy data (for demonstration purposes)
def simulate_occupancy_data():
    # In this example, the grid is empty (all cells unoccupied).
    pass

# Update occupancy grid with simulated data
simulate_occupancy_data()

# Create black borders
occupancy_grid[:border_width, :] = 100
occupancy_grid[-border_width:, :] = 100
occupancy_grid[:, :border_width] = 100
occupancy_grid[:, -border_width:] = 100

# Main loop
while not rospy.is_shutdown():
    # Publish ROS occupancy grid message
    grid_msg = OccupancyGrid()
    grid_msg.header.stamp = rospy.Time.now()
    grid_msg.header.frame_id = 'map'
    grid_msg.info.width = grid_size[1]
    grid_msg.info.height = grid_size[0]
    grid_msg.info.resolution = grid_resolution
    grid_msg.info.origin.position.x = -map_size[1] / 2
    grid_msg.info.origin.position.y = -map_size[0] / 2
    grid_msg.data = list(occupancy_grid.flatten())
    pub.publish(grid_msg)

    rate.sleep()

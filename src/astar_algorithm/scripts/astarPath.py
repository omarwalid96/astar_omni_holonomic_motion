#!/usr/bin/env python3

import math
import random
import sys
from astar_algorithm.cell import Cell  
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
import time
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header

class AStarNode(Node):
    def __init__(self):
        """!__init__ AstarNode init
        """
        super().__init__('astar')
        self.get_logger().info("---------------------A* Node Initalized----------------------")

        self.grid = []
        self.width = 25
        self.height = 25
        self.grid_resolution = 0.05
        self.grid_rows = int(self.width / self.grid_resolution)
        self.grid_cols = int(self.height / self.grid_resolution)
        self.use_diagonal = True

        self.robot_position_x = 0
        self.robot_position_y = 0
        self.goal_position_x = 15
        self.goal_position_y = 15

        self.start = None
        self.end = None
        self.open_list = []
        self.closed_list = []
        self.path = []

        self.init_grid()

        self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.goal_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.path_publisher = self.create_publisher(Path, '/path', 10)
        self.marker_pub = self.create_publisher(Marker, '/goal_marker', 1)

    def init_grid(self):
        """!init_grid Initalize grid based on task input
        """
        for x in range(self.grid_cols):
            row_list = []
            for y in range(self.grid_rows):
                row_list.append(Cell(x, y))
            self.grid.append(row_list)

        for x in range(self.grid_cols):
            for y in range(self.grid_rows):
                self.grid[x][y].addNeighbors(self.grid, self.grid_cols, self.grid_rows, self.use_diagonal)
                self.grid[x][y].wall = False

    def heuristic(self, current_node, end_node):
        """!heuristic calculate cost based on educlidean distance from current Cell to end

        @param current_node: current cell @type current_node: Cell
        @param end_node: final cell @type end_node: Cell
        @return: cost @rtype: float
        """
        return math.sqrt((current_node.x_pos - end_node.x_pos)**2 + abs(current_node.y_pos - end_node.y_pos)**2)

    def a_star_backtrack(self, current):
        """!a_star_backtrack Back track discovered path, and publish trajectory
        @param current: current Cell @type current: Cell
        """
        in_path = current
        while True:
            if in_path.previous_node is None or in_path.previous_node == self.start:
                break
            else:
                self.path.append(in_path.previous_node)
                in_path = in_path.previous_node
        self.path.reverse()

        p = Path()
        p.header.frame_id = 'map'

        for i in self.path:
            current_p = PoseStamped()
            current_p.header.frame_id = 'map'
            current_p.pose.position.x = i.x_pos * self.grid_resolution
            current_p.pose.position.y = i.y_pos * self.grid_resolution
            current_p.pose.orientation.w = 1.0
            p.poses.append(current_p)

        self.path_publisher.publish(p)
        self.start = None
        self.end = None
        self.open_list = []
        self.closed_list = []
        self.path = []

    def a_star_search(self):
        """!a_star_search Main A* algorithm logic, checks free and occupied cells, loops overs the cells based
        on the cost and when the end goal is reached, the back track function is called to assure the path
        """
        self.start.h_score = self.heuristic(self.start, self.end)
        self.open_list.append(self.start)

        while len(self.open_list) > 0:
            self.open_list.sort(key=lambda x: x.f_score)
            current_node = self.open_list[0]

            if current_node == self.end:
                self.open_list.remove(current_node)
                self.a_star_backtrack(current_node)
                return

            else:
                self.open_list.remove(current_node)
                self.closed_list.append(current_node)

                for cells in current_node.neighbors:
                    if cells in self.closed_list or cells.wall:
                        continue
                    else:
                        new_g_score = current_node.g_score + 1
                        use_new_path = False

                        if cells in self.open_list:
                            if new_g_score < cells.g_score:
                                cells.g_score = new_g_score
                                use_new_path = True
                        else:
                            cells.g_score = new_g_score
                            use_new_path = True
                            self.open_list.append(cells)

                        if use_new_path:
                            cells.h_score = self.heuristic(cells, self.end)
                            cells.f_score = cells.g_score + cells.h_score
                            cells.previous_node = current_node

    def goal_callback(self, msg):
        """!goal_callback Subscribes to  /move_base_simple/goal and gets data from rviz

        @param msg: goal data @type msg: PoseStamped
        """
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        self.end = self.grid[int(goal_x / self.grid_resolution)][int(goal_y / self.grid_resolution)]
        p=Point()
        p.x=goal_x
        p.y=goal_y
        
        self.publish_goal_marker(p)
        self.a_star_search()

    def odom_callback(self, msg):
        """!odom_callback Subscribes to /odom to get current robot pose

        @param msg: robot current pose @type msg: Odometry
        """
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        self.start = self.grid[int(pos_x / self.grid_resolution)][int(pos_y / self.grid_resolution)]

    def publish_goal_marker(self, point):
        """!publish_goal_marker Publish marker for the desired goal /goal_marker

        @param point: goal pose @type point: Point
        """
        goal_marker = Marker()
        goal_marker.header = Header()
        goal_marker.header.frame_id = 'map'
        goal_marker.id = 0
        goal_marker.type = Marker.CYLINDER
        goal_marker.action = Marker.ADD
        goal_marker.pose.position = point
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale.x = 0.5
        goal_marker.scale.y = 0.5
        goal_marker.scale.z = 0.001
        goal_marker.color.a = 1.0
        goal_marker.color.r = 0.0
        goal_marker.color.g = 1.0
        goal_marker.color.b = 0.0
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(goal_marker)

def main(args=None):
    rclpy.init(args=args)
    astar_node = AStarNode()
    rclpy.spin(astar_node)
    astar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

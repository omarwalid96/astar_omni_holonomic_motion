#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

from std_msgs.msg import Header
import numpy as np
import math
from dynamic_reconfigure.server import Server
from astar.cfg import WaypointFollowerConfig 

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        
    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class WaypointFollower:
    def __init__(self):
        rospy.init_node('waypoint_follower')
        
        self.path = None
        self.current_waypoint = 0
        self.robot_pose = None

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_subscriber = rospy.Subscriber('/path', Path, self.path_callback)
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.pose_callback)
         # Initialize PID controllers for linear x and linear y
        self.pid_linear_x = PIDController(kp=28.0, ki=0.0, kd=22.0)
        self.pid_linear_y = PIDController(kp=28.0, ki=0.0, kd=22.0)
        self.server = Server(WaypointFollowerConfig, self.reconfigure_callback)

    def reconfigure_callback(self, config, level):
        # Update PID gains based on dynamic reconfigure parameters

        self.pid_linear_x =PIDController(kp=config.kp_linear, ki=config.ki_linear, kd=config.kd_linear)
        self.pid_linear_y =PIDController(kp=config.kp_linear, ki=config.ki_linear, kd=config.kd_linear)
        
        return config
    def path_callback(self, path_msg):
        print("Length of path ",len(path_msg.poses))
        self.path = path_msg.poses
        self.current_waypoint = 0
    
    def pose_callback(self, pose_msg):
        self.robot_pose = pose_msg.pose.pose
        # print(self.robot_pose.position.x)

        
   
    def run(self):
        rate = rospy.Rate(500)  # 10 Hz
        
        while not rospy.is_shutdown():
            if self.path is not None and self.robot_pose is not None:
                if len(self.path) <= 1:
                    self.path = None
                    self.current_waypoint = 0
                    self.path=None
                    twist_cmd = Twist()
                    twist_cmd.linear.x = 0.0
                    twist_cmd.angular.y = 0.0
                    
                    self.velocity_publisher.publish(twist_cmd)
                    continue

                current_waypoint = self.path[self.current_waypoint]
                distance_to_waypoint = np.sqrt((current_waypoint.pose.position.x - self.robot_pose.position.x)**2 +
                                               (current_waypoint.pose.position.y - self.robot_pose.position.y)**2)
                
                if distance_to_waypoint < 0.1:  # If close to the waypoint, move to the next one
                    self.current_waypoint += 1
                    print("Waypoint finished : ",self.current_waypoint)
                    if self.current_waypoint >= len(self.path):
                        self.path = None
                        self.current_waypoint = 0
                        self.path=None
                        twist_cmd = Twist()
                        twist_cmd.linear.x = 0.0
                        twist_cmd.angular.y = 0.0
                        
                        self.velocity_publisher.publish(twist_cmd)
                        continue
                
                max_linear_velocity = 0.7
                distance_to_waypoint_x = current_waypoint.pose.position.x - self.robot_pose.position.x
                distance_to_waypoint_y = current_waypoint.pose.position.y - self.robot_pose.position.y
                
                # # Calculate desired linear velocity for the x-axis based on the x distance error
                desired_linear_velocity_x = self.pid_linear_x.compute(distance_to_waypoint_x)
                
                # Calculate desired linear velocity for the y-axis based on the y distance error
                desired_linear_velocity_y = self.pid_linear_y.compute(distance_to_waypoint_y)
                
                # Limit the maximum linear velocities
                if abs(desired_linear_velocity_x) > max_linear_velocity:
                    desired_linear_velocity_x = max_linear_velocity * np.sign(desired_linear_velocity_x)
                if abs(desired_linear_velocity_y) > max_linear_velocity:
                    desired_linear_velocity_y = max_linear_velocity * np.sign(desired_linear_velocity_y)
                
                twist_cmd = Twist()
                twist_cmd.linear.x = desired_linear_velocity_x
                twist_cmd.linear.y = desired_linear_velocity_y
                
                self.velocity_publisher.publish(twist_cmd)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        waypoint_follower = WaypointFollower()
        waypoint_follower.run()
    except rospy.ROSInterruptException:
        pass

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
import time
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
        self.command_index = 0
        self.current_waypoint = 0
        self.velocityList=[]
        self.robot_pose = None
        self.robot_vel = None
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_subscriber = rospy.Subscriber('/path', Path, self.path_callback)
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.pose_callback)
         # Initialize PID controllers for linear x and linear y
        self.pid_linear_x = PIDController(kp=2.0, ki=0.0, kd=1.5)
        self.pid_linear_y = PIDController(kp=2.0, ki=0.0, kd=1.5)
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
        self.generate_smooth_trajectory()
    
    def pose_callback(self, pose_msg):
        self.robot_pose = pose_msg.pose.pose
        self.robot_vel = pose_msg.twist.twist
        # print(self.robot_pose.position.x)

    def generate_smooth_trajectory(self):
        if self.path is None:
            return
        
        # for i in range(len(self.path) - 1):
        current_waypoint = self.path[0].pose.position
        next_waypoint = self.path[len(self.path) - 1].pose.position
        
        distance = np.sqrt((next_waypoint.x - current_waypoint.x)**2 +
                        (next_waypoint.y - current_waypoint.y)**2)
        
        # Calculate smooth velocity profile parameters
        max_velocity = 0.7  # Adjust as needed
        max_acceleration = 0.2  # Adjust as needed
        max_deceleration = 0.32  # Adjust as needed
        
        # Calculate time intervals for acceleration, constant velocity, and deceleration
        accel_time = max_velocity / max_acceleration
        decel_time = max_velocity / max_deceleration
        
        # Calculate segment time based on distances and max velocity
        segment_time = distance / max_velocity
        
        # Adjust segment time to account for acceleration and deceleration
        if segment_time > (accel_time + decel_time):
            segment_time -= (accel_time + decel_time)
        else:
            # If segment is too short for acceleration and deceleration, adjust
            accel_time = segment_time / 2.0
            decel_time = segment_time / 2.0
        print(accel_time,segment_time,decel_time)
        # Generate and publish velocity commands for the segment
        self.velocityList = self.generate_segment_velocity_commands(
            distance, accel_time, segment_time, decel_time
        )
        # self.executePath(self.velocityList)
        # for i in velocity_commands:
        #     print(i.linear.x,i.linear.y)
        # print(velocity_commands)
        self.path=None
    def generate_segment_velocity_commands(self, distance, accel_time, segment_time, decel_time):
        # Parameters for linear velocity profile
        initial_velocity = 0.0
        final_velocity = 0.0
        max_velocity = 0.7  # Adjust as needed
        max_acceleration = max_velocity / accel_time
        max_deceleration = max_velocity / decel_time

        # Time intervals for each phase
        t1 = accel_time
        t2 = segment_time - (accel_time + decel_time)
        t3 = decel_time

        velocity_commands = []

        # Acceleration phase
        for t in np.arange(0, t1, 0.01):
            v = initial_velocity + max_acceleration * t
            if v > max_velocity:
                v = max_velocity
            cmd = Twist()
            cmd.linear.x = v
            cmd.linear.y = v  # Adjust as needed for linear y
            velocity_commands.append(cmd)

        # Constant velocity phase
        for t in np.arange(0, t2, 0.01):
            cmd = Twist()
            cmd.linear.x = max_velocity
            cmd.linear.y = max_velocity  # Adjust as needed for linear y
            velocity_commands.append(cmd)

        # Deceleration phase
        for t in np.arange(0, t3, 0.01):
            v = max_velocity - max_deceleration * t
            if v < 0:
                v = 0
            cmd = Twist()
            cmd.linear.x = v
            cmd.linear.y = v  # Adjust as needed for linear y
            velocity_commands.append(cmd)
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = 0  # Adjust as needed for linear y
        velocity_commands.append(cmd)
        print("Finished Deceleration last vel ",velocity_commands[len(velocity_commands)-1])

        return velocity_commands
    def executePath(self,velocity):
        if self.command_index < len(self.velocityList):
            cmd = self.velocityList[self.command_index]

            # Compare actual robot velocity with desired velocity
            error_x = cmd.linear.x - self.robot_vel.linear.x
            error_y = cmd.linear.y - self.robot_vel.linear.y

            # Calculate control output for adjusting velocity
            proportional_gain = 0.5  # Adjust as needed
            control_output_x = proportional_gain * error_x
            control_output_y = proportional_gain * error_y

            # Adjust linear x and linear y velocities based on control output
            twist_cmd = Twist()
            twist_cmd.linear.x = cmd.linear.x + control_output_x
            twist_cmd.linear.y = cmd.linear.y + control_output_y

            self.velocity_publisher.publish(twist_cmd)

            # Check if the robot has reached the current velocity command
            if abs(error_x) < 0.01 and abs(error_y) < 0.01:
                self.command_index += 1
        
        time.sleep(0.01)

    def run(self):
        rate = rospy.Rate(100)  # 10 Hz
        
        while not rospy.is_shutdown():
            
            self.executePath(self.velocityList)
            rate.sleep()

if __name__ == '__main__':
    try:
        waypoint_follower = WaypointFollower()
        waypoint_follower.run()
    except rospy.ROSInterruptException:
        pass

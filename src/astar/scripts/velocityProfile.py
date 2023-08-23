#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import numpy as np
import math

class SmoothTrajectoryPlanner:
    def __init__(self):
        rospy.init_node('smooth_trajectory_planner')
        
        self.path = None
        self.robot_pose = None

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_subscriber = rospy.Subscriber('/path', Path, self.path_callback)
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.pose_callback)
        
        # Set up other parameters, controllers, etc.
        
    def path_callback(self, path_msg):
        self.path = path_msg.poses
        print("Got Path")
    
    def pose_callback(self, pose_msg):
        self.robot_pose = pose_msg.pose.pose
        
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
        velocity_commands = self.generate_segment_velocity_commands(
            distance, accel_time, segment_time, decel_time
        )
        # self.publish_velocity_commands(velocity_commands)
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
        
    def publish_velocity_commands(self, velocity_commands):
        for cmd in velocity_commands:
            self.velocity_publisher.publish(cmd)
            # rospy.sleep(0.1)  # Adjust sleep duration
        
    def run(self):
        rate = rospy.Rate(100)  # 10 Hz
        
        while not rospy.is_shutdown():
            self.generate_smooth_trajectory()
            rate.sleep()

if __name__ == '__main__':
    try:
        planner = SmoothTrajectoryPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass

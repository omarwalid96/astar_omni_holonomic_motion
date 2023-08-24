#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration,Command

def generate_launch_description():


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package="map_publisher",
            executable="mapPublisher.py",
            name="mapPublisher"),

        Node(
            package="map_publisher",
            executable="robotFootprint.py",
            name="robotFootprint"),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_tf", arguments=[
                "--frame-id", "map",        
                "--child-frame-id", "odom"  
            ])
    ])
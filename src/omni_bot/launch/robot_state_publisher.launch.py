#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

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

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file="main.xacro"
    desc_path= os.path.join(get_package_share_directory("omni_bot"),"urdf",urdf_file )

    urdf = os.path.join(get_package_share_directory("omni_bot"),"urdf",urdf_file )
    jointConf = os.path.join(get_package_share_directory("omni_bot"),"comfig","omni_bot.yaml" )

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(get_package_share_directory('omni_bot'), 'config/astar.rviz'),
            description='Path to RViz configuration file'),
        
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            emulate_tty=True,
            parameters=[{"use_sim_time":True,"robot_description": Command(["xacro ",desc_path])}],
             arguments=[urdf]),


        Node(
            package="omni_bot",
            executable="mapPublisher.py",
            name="mapPublisher"),

        
        Node(
            package="omni_bot",
            executable="robotFootprint.py",
            name="robotFootprint"),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_tf", arguments=[
                "--frame-id", "map",        
                "--child-frame-id", "odom"  
            ]),

        Node(
            package="omni_bot",
            executable="astarPath.py",
            name="astar"),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                '-d', LaunchConfiguration('rviz_config')
            ],
        ),

        Node(
            package="omni_bot",
            executable="waypointCommands.py",
            name="waypointCommands",
        )

    ])
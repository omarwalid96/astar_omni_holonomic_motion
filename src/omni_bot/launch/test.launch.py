import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, ExecuteProcess
from launch.substitutions import LaunchConfiguration,Command
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node
def generate_launch_description():
    urdf_file="main.xacro"
    desc_path= os.path.join(get_package_share_directory("omni_bot"),"urdf",urdf_file )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        emulate_tty=True,
        parameters=[{"use_sim_time":True,"robot_description": Command(["xacro ",desc_path])}],
        output="screen"
    )
    return LaunchDescription([
        # Include Gazebo empty world launch
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [get_package_share_directory('gazebo_ros'), '/launch/', 'gazebo.launch.py']
            ),
            launch_arguments={
                'world_name': '',
                'paused': 'false',
                'use_sim_time': 'true',
                'gui': 'true',
                'headless': 'false',
                'verbose': ''
            }.items()
        ),
        robot_state_publisher
        
        # # Declare robot_description argument
        # DeclareLaunchArgument('robot_description', default_value=''),
        # DeclareLaunchArgument('robot_urdf', default_value=[get_package_share_directory('omni_bot'), 'urdf', 'main.xacro']),

        # # Spawn the robot model
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
        #         '-file', '/home/omni_ws/install/omni_bot/share/omni_bot/urdf/main.xacro', '-urdf', '-model', 'wheel'
        #     ],
        #     output='screen',
        # ),

        # # Load robot parameters from YAML file
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'param', 'load', 'omni_bot', '/home/omni_ws/install/omni_bot/share/omni_bot/config/omni_bot.yaml'
        #     ],
        #     output='screen',
        # ),

        # # Spawn controller nodes
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'run', 'controller_manager', 'spawner',
        #         '--', 'joint_state_controller', 'left_wheel_controller', 'right_wheel_controller', 'front_wheel_controller'
        #     ],
        #     output='screen',
        #     emulate_tty=True,
        # ),

        # # Start robot state publisher
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher','/home/omni_ws/install/omni_bot/share/omni_bot/urdf/main.xacro', '-n', 'omni_bot'
        #     ],
        #     output='screen',
        #     emulate_tty=True,
        # )
    ])
    
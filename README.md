
<h1 align="center">Zal Robotics Engineer Task</h1>

----
## 📝Table of Contents
- [📝Table of Contents](#table-of-contents)
- [Code Docs](#code-docs)
- [Solution](#solution)
- [Instillation](#instillation)
  - [Ros2 Humble](#ros2-humble)
  - [Docker](#docker)
- [🎈 Usage ](#-usage-)
  - [Ros2 Humble](#ros2-humble-1)
  - [Docker](#docker-1)
- [✍️ Authors ](#️-authors-)
- [🏁 Getting Started ](#-getting-started-)
  - [Problem](#problem)
  - [Shortest Path (A\*)](#shortest-path-a)
  - [Controller (navigation\_control)](#controller-navigation_control)
  - [Map (map\_publisher)](#map-map_publisher)
  - [Odometry](#odometry)
    - [Calculated with Velocity Vectors](#calculated-with-velocity-vectors)
    - [Simulation](#simulation)
  - [Simulation](#simulation-1)
- [Directory Tree](#directory-tree)

## Code Docs
https://omarwalid96.gitlab.io/zal_robotics_engineer_task/


## Solution
[Video](https://imgur.com/yh2QjKe)
![Video](https://imgur.com/yh2QjKe)
![Path](https://imgur.com/EWTWwQT)
## Instillation

### Ros2 Humble
`git clone https://oauth2:glpat-ENRdy37JunsqwTFEqnYb@gitlab.com/omarwalid96/zal_robotics_engineer_task.git`
 
### Docker
1. install noVNC `docker pull theasp/novnc:latest`
2. run `docker run -d --rm --net=ros  --env="DISPLAY_WIDTH=3000" --env="DISPLAY_HEIGHT=1800" --env="RUN_XTERM=no" --name=novnc -p=8080:8080  theasp/novnc:latest`
3. opn viewer on any browser `http://localhost:8080/vnc.html`
4. install docker image `docker pull omarwalid96/ros2-zal-task`



## 🎈 Usage <a name="usage"></a>

- Gui opens and you can instantly send commands from **2d goal pose** ![goal](https://imgur.com/gsQyRdB)

### Ros2 Humble
1. `cd working_dir/zal_robotics_engineer_task`
2. `source /opt/ros/humble/setup.bash; source ./install/setup.bash; ros2 launch omni_bot zal_task.launch.py`
### Docker
1. run `docker run -d --rm --net=ros  --env="DISPLAY_WIDTH=3000" --env="DISPLAY_HEIGHT=1800" --env="RUN_XTERM=no" --name=novnc -p=8080:8080  theasp/novnc:latest`
2. run `docker run  -it --net="ros"  --privileged omarwalid96/ros2-zal-task /bin/bash -c "sleep 2;cd /home/zal_robotics_engineer_task ; source /opt/ros/humble/setup.bash; source ./install/setup.bash; ros2 launch omni_bot zal_task.launch.py"`
3. connect to viewer on `http://localhost:8080/vnc.html`
![Connect](https://imgur.com/ohAMiEX)


## ✍️ Authors <a name = "authors"></a>

- [@omarwalid96](https://gitlab.com/omarwalid96) 


## 🏁 Getting Started <a name = "getting_started"></a>
### Problem 
- Design a motion Controller
- Continous Feedback from robot for movment
- Shortest path to Goal
- GUI to support solution
- Send Multiple Goals


### Shortest Path (A*)
Used A* algorith based on Educlidian distance as a heuristic function
**Heuristics to estimate the cost of reaching the goal from each point and explores the most promising paths first.**

- **To Test algorithm indepndantly you can run** `cd src/astar_algorithm/scripts/Pathfinding_Visualizer ; python3 main.py` then follow the steps.
### Controller (navigation_control)
From The Distance Trajectory Provided from the A* Planner, A PID Controller was designed to calculate the required velocity to send to the robot motors to reach points.

- Checks if there is a valid path and robot pose.
- If the path length is less than or equal to 1, it stops the robot.
- Otherwise, it calculates the distance to the current waypoint.
- If the distance is less than a threshold, it moves to the next waypoint.
- If the robot is at the end of the path, it stops the robot.
- Otherwise, it calculates desired linear velocities using PID controllers for the x and y directions.
- It limits the maximum linear velocities and publishes the calculated twist command.

***A dynamic Reconfiguration for the PID Controllers is implmented in the other melodic branch***


### Map (map_publisher)
- Map is designed as per task and published as an occupancy grid map.
- A Transformation also works in the background to link the world together.
  
### Odometry 

Two Models are side by side, one publishes on `/odom` and the other on `/odom_calc`
#### Calculated with Velocity Vectors

From the data taken from the `/cmd_vel` topic, we can calculate Velocities for each motor and Also Odometry.

<img src="https://camo.githubusercontent.com/7c70deb1b3ffd595c18b1bad558f71fd7997743fa90d7db69a25632aa4af92a0/68747470733a2f2f63646e2e7261776769742e636f6d2f4775695269747465722f4f70656e426173652f6d61737465722f696d616765732f67656f6d657472792e737667" alt="Image" style="width:50%; height:50%;">


![0](https://camo.githubusercontent.com/6b5f3b9b567fb03759a116b4105aef5526cd4fa437dc8ab80d13710236d66863/68747470733a2f2f63646e2e7261776769742e636f6d2f4775695269747465722f4f70656e426173652f6d61737465722f696d616765732f666f72776172645f6d6f62696c652e737667)

![1](https://camo.githubusercontent.com/039cb0813a911e98457ce68f56ff347b53b3c9216adaaa86c807e86d6d827fd6/68747470733a2f2f63646e2e7261776769742e636f6d2f4775695269747465722f4f70656e426173652f6d61737465722f696d616765732f666f72776172645f776f726c642e737667)

![2](https://camo.githubusercontent.com/c52ee0855fdde637492baf88954c56196775cf4151b0b044204d49f36e69317a/68747470733a2f2f63646e2e7261776769742e636f6d2f4775695269747465722f4f70656e426173652f6d61737465722f696d616765732f696e76657273655f776f726c642e737667)

![3](https://camo.githubusercontent.com/1c9cb79ba77ccf0410050b643ecb82ae7cc8ca1abcc27fd0e3c9b4b5d183530e/68747470733a2f2f63646e2e7261776769742e636f6d2f4775695269747465722f4f70656e426173652f6d61737465722f696d616765732f696e76657273655f6d6f62696c652e737667)

#### Simulation
Using `ros_planar_move` plugin, Provides `odom` data directly from simulation.

**Description:** model plugin that allows objects to be moved along a horizontal plane using a geometry_msgs/Twist message. The plugin works by imparting a linear velocity (XY) and an angular velocity (Z) to the object every cycle.

### Simulation
Simulated Model is a 3WD Omni robot model from here: https://github.com/GuiRitter/OpenBase

![Gazebo GUI with OpenBase robot](https://github.com/GuiRitter/OpenBase/raw/master/images/Gazebo_GUI.png)

Simulation Modified to work with **ros2** and also holonomic motion plugin was added.


## Directory Tree
```bash
.
├── Doxyfile
├── README.md
├── src
│   ├── astar_algorithm
│   │   ├── astar_algorithm
│   │   │   ├── cell.py
│   │   │   └── __init__.py
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   └── astar.launch.py
│   │   ├── package.xml
│   │   └── scripts
│   │       ├── astarPath.py
│   │       └── Pathfinding_Visualizer
│   │           ├── cell.py
│   │           ├── images
│   │           │   ├── exit_screen.jpg
│   │           │   ├── no_solution.png
│   │           │   ├── screen_1.jpg
│   │           │   ├── screen_2.jpg
│   │           │   ├── screen_3.png
│   │           │   ├── screen_4.jpg
│   │           │   ├── screen_5.jpg
│   │           │   ├── screen_6.jpg
│   │           │   └── screen_7.png
│   │           ├── main.py
│   ├── map_publisher
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   └── map_publisher.launch.py
│   │   ├── map_publisher
│   │   │   └── __init__.py
│   │   ├── package.xml
│   │   └── scripts
│   │       ├── mapPublisher.py
│   │       └── robotFootprint.py
│   ├── navigation_control
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   └── navigation_control.launch.py
│   │   ├── navigation_control
│   │   │   └── __init__.py
│   │   ├── package.xml
│   │   └── scripts
│   │       └── waypointCommands.py
│   └── omni_bot
│       ├── CMakeLists.txt
│       ├── config
│       │   ├── astar.rviz
│       │   └── omni_bot.yaml
│       ├── launch
│       │   ├── robot_state_publisher.launch.py
│       │   ├── spawn_robot.launch.py
│       │   └── zal_task.launch.py
│       ├── meshes
│       │   └── chassis.STL
│       ├── msg
│       │   └── OmniDrive.msg
│       ├── omni_bot
│       │   └── __init__.py
│       ├── package.xml
│       ├── scripts
│       │   └── omniController.py
│       └── urdf
│           ├── main.xacro
│           ├── plugins.gazebo
│           ├── rim.xacro
│           └── roller.xacro
└── style.css
```
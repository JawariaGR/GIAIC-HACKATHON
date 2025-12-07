# Lab 10: Autonomous Navigation with Nav2 in Isaac Sim

## Objective
In this lab, you will learn how to configure and launch the ROS 2 Navigation Stack (Nav2) for a mobile robot within NVIDIA Isaac Sim. You will load a pre-built map, localize your robot, and command it to autonomously navigate to various goal poses, observing its behavior in a high-fidelity simulation.

**Note**: Setting up Nav2 and its integration with Isaac Sim involves multiple configuration files and is quite complex. This lab will focus on guiding you through the process and will heavily reference NVIDIA's official Isaac Sim documentation for Nav2.

## Prerequisite Skills
-   A working ROS 2 Humble installation.
-   Completion of Lab 8 (familiarity with Isaac Sim and ROS 2 integration).
-   Basic understanding of robot perception and SLAM (from Chapter 9).
-   Familiarity with ROS 2 launch files and parameters.

---

## Part 1: Setting Up the Isaac Sim Environment for Nav2

### 1. Launch Isaac Sim

1.  Open the NVIDIA Omniverse Launcher and launch Isaac Sim.
2.  Open a new stage (`File > New Stage`).

### 2. Load a Nav2-Ready Robot and Environment

Isaac Sim provides specific examples for Nav2.

1.  **Load a pre-configured Nav2 scene**:
    -   Go to `File > Open`.
    -   Navigate to the Isaac Sim samples: `omniverse://localhost/NVIDIA/Assets/Isaac/2023.1/Isaac/Environments/Nav/` (path might vary slightly by version).
    -   Open a scene like `Nav_Cube.usd` or a more complex environment such as `Nav_Warehouse.usd`.
2.  **Add a Nav2-ready robot**:
    -   If the scene doesn't include a robot, add one from `Isaac > Robots > ROS_Mobile_Robot` or `Isaac > Robots > Jackal_ROS2`.
    -   Ensure the robot is configured with the necessary ROS 2 Nav2 extensions (e.g., `omni.isaac.ros2_nav2`, `omni.isaac.ros2_imu`, `omni.isaac.ros2_laser_scan`). These extensions are usually enabled automatically when you load a Nav2 sample scene or robot.

## Part 2: Map Generation and Loading

For autonomous navigation, Nav2 requires a map.

### Option A: Using a Pre-built Map (Recommended for Lab)

Many Isaac Sim Nav2 sample scenes come with a pre-generated map (`.yaml` and `.pgm` files).

1.  Locate these map files in the Isaac Sim sample directory (e.g., in a `maps` subfolder within the Nav2 scene's assets).
2.  Copy these map files to your ROS 2 workspace (e.g., `~/ros2_ws/src/my_nav2_config/maps/`).

### Option B: Generating a Map (Advanced)

If you need to generate a map of a custom environment:

1.  **Launch a SLAM algorithm**:
    -   Start Isaac Sim and press "Play".
    -   In a ROS 2 terminal, launch a SLAM algorithm (e.g., `slam_toolbox`) that subscribes to the robot's laser scan topic (`/scan`) and odometry (`/odom`).
    -   Manually drive your robot around the environment in Isaac Sim to explore and build the map.
2.  **Save the map**: Once the environment is mapped, use the `map_saver` tool:
    ```bash
    ros2 run nav2_map_server map_saver_cli -f my_isaac_map
    ```
    This will save `my_isaac_map.yaml` and `my_isaac_map.pgm` files.

## Part 3: Launching the Nav2 Stack

Nav2 typically runs as a collection of nodes orchestrated by a launch file.

1.  **Install Nav2**: If not already installed, install the Nav2 meta-package:
    ```bash
    sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
    ```
2.  **Create a Nav2 Launch File**: You'll need a launch file that configures Nav2 for your robot and loads your map. NVIDIA provides example launch files for their robots within Isaac Sim, which you can adapt.
    -   Refer to the Isaac Sim Nav2 documentation for specific launch file examples and required parameters: [https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/nav_ros2_sensors.html](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/nav_ros2_sensors.html)
    -   Key elements in the launch file will include:
        -   `map_server` node: Loads your `my_isaac_map.yaml`.
        -   `amcl` node: For localization on the map.
        -   `nav2_controller` (global and local planners).
        -   `rviz2` (with a pre-configured Nav2 view).
        -   Crucially, set `use_sim_time` to `True` for all Nav2 nodes.

### Example Nav2 Launch (Conceptual)

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    pkg_name = 'my_nav2_config' # Your package containing maps and Nav2 params
    pkg_dir = get_package_share_directory(pkg_name)

    map_file = os.path.join(pkg_dir, 'maps', 'my_isaac_map.yaml')
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml') # Your Nav2 config

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('map', default_value=map_file, description='Full path to map file to load'),
        DeclareLaunchArgument('params_file', default_value=nav2_params_file, description='Full path to param file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file')
            }.items(),
        ),
        # You might also include an RViz2 node here with a pre-configured view
    ])
```

## Part 4: Autonomous Navigation

1.  **Launch Isaac Sim**: Start Isaac Sim and load your Nav2-ready robot and environment. Press "Play" to start the simulation.
2.  **Launch the Nav2 Stack**:
    -   In a new terminal (after sourcing your ROS 2 workspace), launch your Nav2 configuration:
        ```bash
        ros2 launch my_nav2_config my_nav2_bringup.launch.py
        ```
    -   This will start all Nav2 nodes and likely RViz2.
3.  **Set Initial Pose (Localization)**:
    -   In RViz2, click the "2D Pose Estimate" button (top toolbar).
    -   Click and drag on the map in RViz to indicate the robot's approximate starting position and orientation. This initializes AMCL.
4.  **Send a Goal Pose**:
    -   In RViz2, click the "2D Goal Pose" button.
    -   Click and drag on the map to set a destination for the robot.
5.  **Observe Navigation**:
    -   Watch the robot in Isaac Sim. It should start planning a path and moving autonomously towards the goal, avoiding obstacles.
    -   Observe the global and local plans, costmaps, and robot trajectory in RViz2.

## Expected Behavior

-   The robot in Isaac Sim successfully localizes itself on the loaded map.
-   When a goal is given, Nav2 plans a collision-free path.
-   The robot autonomously drives to the goal, reacting to any dynamic obstacles.
-   You can monitor the robot's progress and the navigation state in RViz2.

**Congratulations! You have successfully configured and used the ROS 2 Nav2 stack to enable autonomous navigation for your robot in Isaac Sim!** You've seen the full power of perception, planning, and control working together.

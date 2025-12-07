# Lab 5: Spawning Your URDF Model in Gazebo

## Objective
In this lab, you will take the URDF model of your 2-DOF robotic arm created in Lab 4 and bring it into the Gazebo physics simulator. You will learn how to launch Gazebo with a basic world and then spawn your robot model into that world using ROS 2 tools.

## Prerequisite Skills
-   Completion of Lab 4 (creating and visualizing a URDF model).
-   A working ROS 2 Humble installation with `gazebo_ros_pkgs` installed.
    ```bash
    sudo apt install ros-humble-gazebo-ros-pkgs
    ```

---

## 1. Create a Simple Gazebo World File

First, let's define a very basic Gazebo world that includes gravity and a flat ground plane.

1.  Navigate to your `my_robot_description` package:
    ```bash
    cd ~/ros2_ws/src/my_robot_description
    ```
2.  Create a new directory named `worlds`:
    ```bash
    mkdir worlds
    ```
3.  Inside the `worlds` directory, create a file named `empty_arm_world.world`.
    ```bash
    code worlds/empty_arm_world.world
    ```
4.  Add the following SDF content to `empty_arm_world.world`:
    ```xml
    <?xml version="1.0" ?>
    <sdf version="1.7">
      <world name="empty_arm_world">
        <gravity>0 0 -9.8</gravity>
        <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
        <atmosphere type='adiabatic'/>
        <physics name='default_physics' default='0' type='ode'>
          <max_step_size>0.001</max_step_size>
          <real_time_factor>1</real_time_factor>
          <real_time_update_rate>1000</real_time_update_rate>
        </physics>
        <scene>
          <ambient>0.4 0.4 0.4 1</ambient>
          <background>0.7 0.7 0.7 1</background>
          <shadows>1</shadows>
        </scene>

        <!-- A simple ground plane -->
        <model name="ground_plane">
          <static>true</static>
          <link name="link">
            <collision name="collision">
              <geometry>
                <plane>
                  <normal>0 0 1</normal>
                  <size>100 100</size>
                </plane>
              </geometry>
              <surface>
                <friction>
                  <ode>
                    <mu>1.0</mu>
                    <mu2>1.0</mu2>
                  </ode>
                </friction>
              </surface>
            </collision>
            <visual name="visual">
              <geometry>
                <plane>
                  <normal>0 0 1</normal>
                  <size>100 100</size>
                </plane>
              </geometry>
              <material>
                <ambient>0.8 0.8 0.8 1</ambient>
                <diffuse>0.8 0.8 0.8 1</diffuse>
                <specular>0.8 0.8 0.8 1</specular>
              </material>
            </visual>
          </link>
        </model>

      </world>
    </sdf>
    ```

## 2. Update `CMakeLists.txt`

We need to tell `colcon` to install our new `worlds` directory.

1.  Open `CMakeLists.txt` in your `my_robot_description` package.
2.  Add `worlds` to the `install(DIRECTORY ...)` command, so it looks like this:
    ```cmake
    install(DIRECTORY
      urdf
      launch
      rviz
      worlds  # Add this line
      DESTINATION share/${PROJECT_NAME}
    )
    ```

## 3. Create a Launch File to Spawn the Robot

We'll create a new launch file that starts Gazebo and then uses a `spawn_entity.py` node to load our URDF into it.

1.  Create a new file named `spawn_arm.launch.py` inside the `launch` directory.
2.  Add the following Python code:

    ```python
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    def generate_launch_description():
        # Get path to your URDF file
        urdf_file_path = os.path.join(
            get_package_share_directory('my_robot_description'),
            'urdf',
            'my_arm.urdf')

        # Get path to your world file
        world_file_path = os.path.join(
            get_package_share_directory('my_robot_description'),
            'worlds',
            'empty_arm_world.world')

        # Read the URDF file content
        with open(urdf_file_path, 'r') as file:
            robot_description = file.read()

        # Gazebo launch
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={'world': world_file_path}.items(),
        )

        # Node for robot_state_publisher
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True # Essential for Gazebo
            }]
        )

        # Node for joint_state_publisher_gui
        jsp_gui_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        )

        # Node for spawning the robot in Gazebo
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                       '-entity', 'my_2_dof_arm',
                       '-x', '0.0', '-y', '0.0', '-z', '0.1'], # Slightly above ground
            output='screen'
        )

        return LaunchDescription([
            gazebo,
            rsp_node,
            jsp_gui_node,
            spawn_entity,
        ])
    ```

## 4. Build and Launch

1.  Navigate to your workspace root (`cd ~/ros2_ws`) and build the package:
    ```bash
    colcon build --packages-select my_robot_description
    ```
2.  Source the overlay:
    ```bash
    . install/setup.bash
    ```
3.  Launch your new file:
    ```bash
    ros2 launch my_robot_description spawn_arm.launch.py
    ```

    This will open the Gazebo simulator, and after a few seconds, your 2-DOF arm model should appear, along with the `Joint State Publisher` GUI.

## 5. Interact with Your Robot in Gazebo

1.  In the Gazebo window, you should see your arm model sitting on the ground plane. You can use your mouse to move the camera around.
2.  Open the `Joint State Publisher` GUI (if it didn't open automatically).
3.  Adjust the sliders for `joint1` and `joint2`. You should see your arm move realistically in the Gazebo simulation, affected by gravity. You can even try to make it fall over!

**Congratulations! You have successfully spawned your URDF model into Gazebo and are controlling its joints in a physics-enabled simulation.** This is a huge step towards developing and testing complex robot behaviors in a safe, virtual environment.

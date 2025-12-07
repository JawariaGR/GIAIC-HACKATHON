# Lab 4: Building and Visualizing Your First URDF

## Objective
In this lab, you will write a URDF file from scratch to model a simple two-degree-of-freedom (2-DOF) robotic arm. You will then use ROS 2 tools like RViz2 and `robot_state_publisher` to visualize your creation and control its joints with a GUI.

## Prerequisite Skills
-   A working ROS 2 Humble installation.
-   Basic understanding of XML syntax is helpful but not required.

---

## 1. Create a New Package

It's good practice to keep robot description files in their own package.

1.  Navigate to the `src` directory of your workspace:
    ```bash
    cd ~/ros2_ws/src
    ```
2.  Create a new package named `my_robot_description`:
    ```bash
    ros2 pkg create --build-type ament_cmake my_robot_description
    ```
    *(Note: We use `ament_cmake` even though we aren't writing C++ code because it's standard for packages that contain non-code assets like URDFs and launch files.)*

3.  Inside this new package, create three new directories: `urdf`, `launch`, and `rviz`.
    ```bash
    cd my_robot_description
    mkdir urdf launch rviz
    ```

## 2. Write the URDF File

Now, let's create the model for our arm.

1.  Create a new file named `my_arm.urdf` inside the `urdf` directory.
2.  Add the following XML code. Read the comments to understand what each section does.

    ```xml
    <?xml version="1.0"?>
    <robot name="my_2_dof_arm">

      <!-- ********** LINKS ********** -->

      <!-- Base Link: The fixed foundation of the arm -->
      <link name="base_link">
        <visual>
          <geometry>
            <cylinder length="0.05" radius="0.1"/>
          </geometry>
          <origin xyz="0 0 0.025" rpy="0 0 0"/>
          <material name="grey">
            <color rgba="0.5 0.5 0.5 1.0"/>
          </material>
        </visual>
      </link>

      <!-- Arm Link 1: The first moving part -->
      <link name="arm_link_1">
        <visual>
          <geometry>
            <box size="0.5 0.1 0.1"/>
          </geometry>
          <origin xyz="0.25 0 0" rpy="0 0 0"/>
          <material name="blue">
            <color rgba="0.0 0.0 0.8 1.0"/>
          </material>
        </visual>
      </link>

      <!-- Arm Link 2: The second moving part -->
      <link name="arm_link_2">
        <visual>
          <geometry>
            <box size="0.5 0.1 0.1"/>
          </geometry>
          <origin xyz="0.25 0 0" rpy="0 0 0"/>
          <material name="green">
            <color rgba="0.0 0.8 0.0 1.0"/>
          </material>
        </visual>
      </link>

      <!-- ********** JOINTS ********** -->

      <!-- Joint 1: Connects base_link to arm_link_1 -->
      <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="arm_link_1"/>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
      </joint>

      <!-- Joint 2: Connects arm_link_1 to arm_link_2 -->
      <joint name="joint2" type="revolute">
        <parent link="arm_link_1"/>
        <child link="arm_link_2"/>
        <origin xyz="0.5 0 0" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
      </joint>

    </robot>
    ```

## 3. Create a Launch File

A launch file allows us to start multiple nodes with a single command. We'll use one to start `robot_state_publisher`, `joint_state_publisher_gui`, and `rviz2`.

1.  Create a file named `display.launch.py` inside the `launch` directory.
2.  Add the following Python code:

    ```python
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch_ros.actions import Node
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration

    def generate_launch_description():
        # Get the path to the URDF file
        urdf_file_path = os.path.join(
            get_package_share_directory('my_robot_description'),
            'urdf',
            'my_arm.urdf')

        # Read the URDF file content
        with open(urdf_file_path, 'r') as file:
            robot_description = file.read()

        return LaunchDescription([
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation (Gazebo) clock if true'),

            # Node for robot_state_publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_description,
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }]
            ),

            # Node for joint_state_publisher_gui
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui'
            ),

            # Node for RViz2
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen'
            )
        ])
    ```

## 4. Update Build and Install Rules

We need to tell `colcon` to install our new `urdf` and `launch` directories.

1.  Open `CMakeLists.txt` in your `my_robot_description` package.
2.  Add these lines to the bottom of the file:
    ```cmake
    install(DIRECTORY
      urdf
      launch
      rviz
      DESTINATION share/${PROJECT_NAME}
    )
    ```

## 5. Build and Launch

1.  Navigate to your workspace root (`cd ~/ros2_ws`) and build the package:
    ```bash
    colcon build --packages-select my_robot_description
    ```
2.  Source the overlay:
    ```bash
    . install/setup.bash
    ```
3.  Use `ros2 launch` to run your new launch file:
    ```bash
    ros2 launch my_robot_description display.launch.py
    ```
    This command will open three windows: RViz, a terminal running the nodes, and a "Joint State Publisher" GUI with sliders.

## 6. Visualize in RViz

1.  In the RViz window (bottom-left panel), change the "Fixed Frame" from `map` to `base_link`.
2.  Click the "Add" button (bottom-left), and in the "By display type" tab, select "RobotModel". Click OK.
3.  You should now see your 2-DOF arm in the RViz window!
4.  Find the "Joint State Publisher" window. Move the sliders for `joint1` and `joint2`. You should see the robot model in RViz move in real-time.

**Congratulations! You have successfully created a URDF file and visualized it in RViz2. You have built a digital representation of a robot and can control its virtual joints.**

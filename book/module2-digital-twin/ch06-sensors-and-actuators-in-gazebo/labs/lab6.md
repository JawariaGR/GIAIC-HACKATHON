# Lab 6: Equipping Your Robot with Sensors and Actuators in Gazebo

## Objective
In this lab, you will design a simple mobile robot with a LiDAR sensor and differential drive actuators. You will integrate this robot into a Gazebo simulation and write a basic ROS 2 Python node to make the robot drive forward, stopping if it detects an obstacle using its LiDAR data.

## Prerequisite Skills
-   Completion of Lab 5 (spawning URDF in Gazebo).
-   Basic ROS 2 Python node development (from Lab 2).
-   Familiarity with URDF (from Lab 4).
-   `gazebo_ros_pkgs` installed.

---

## 1. Create a New Robot Package

We'll create a dedicated package for our mobile robot.

1.  Navigate to your workspace `src` directory:
    ```bash
    cd ~/ros2_ws/src
    ```
2.  Create a new `ament_cmake` package named `my_mobile_robot`:
    ```bash
    ros2 pkg create --build-type ament_cmake my_mobile_robot
    ```
3.  Inside `my_mobile_robot`, create `urdf`, `launch`, `worlds`, and `python` directories:
    ```bash
    cd my_mobile_robot
    mkdir urdf launch worlds python
    ```
4.  Inside `my_mobile_robot/python`, create a new folder `my_mobile_robot` (this is necessary for `ament_python` entry points):
    ```bash
    mkdir python/my_mobile_robot
    ```

## 2. Define the Mobile Robot URDF with Sensors and Actuators

This URDF will define our robot's physical structure, its wheels, a LiDAR sensor, and integrate Gazebo plugins.

1.  Create `my_mobile_robot/urdf/mobile_robot.urdf`:
    ```xml
    <?xml version="1.0"?>
    <robot name="my_mobile_robot">

      <!-- BASE LINK -->
      <link name="base_link">
        <visual>
          <geometry><box size="0.4 0.2 0.1"/></geometry>
          <material name="blue"><color rgba="0 0 1 1"/></material>
        </visual>
        <collision><geometry><box size="0.4 0.2 0.1"/></geometry></collision>
        <inertial>
          <mass value="5.0"/>
          <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.04"/>
        </inertial>
      </link>

      <!-- CASTER WHEEL -->
      <link name="caster_wheel_link">
        <visual>
          <geometry><sphere radius="0.05"/></geometry>
          <material name="black"><color rgba="0 0 0 1"/></material>
        </visual>
        <collision><geometry><sphere radius="0.05"/></geometry></collision>
        <inertial>
          <mass value="0.1"/>
          <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
        </inertial>
      </link>
      <joint name="caster_wheel_joint" type="fixed">
        <parent link="base_link"/>
        <child link="caster_wheel_link"/>
        <origin xyz="-0.15 0 -0.05" rpy="0 0 0"/>
      </joint>

      <!-- LEFT WHEEL -->
      <link name="left_wheel_link">
        <visual>
          <origin xyz="0 0 0" rpy="1.5707 0 0"/>
          <geometry><cylinder radius="0.05" length="0.02"/></geometry>
          <material name="black"/>
        </visual>
        <collision>
          <origin xyz="0 0 0" rpy="1.5707 0 0"/>
          <geometry><cylinder radius="0.05" length="0.02"/></geometry>
        </collision>
        <inertial>
          <mass value="0.2"/>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <inertia ixx="0.0002" ixy="0" ixz="0" iyy="0.0002" iyz="0" izz="0.0004"/>
        </inertial>
      </link>
      <joint name="left_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="left_wheel_link"/>
        <origin xyz="0 0.11 -0.05" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
      </joint>

      <!-- RIGHT WHEEL -->
      <link name="right_wheel_link">
        <visual>
          <origin xyz="0 0 0" rpy="1.5707 0 0"/>
          <geometry><cylinder radius="0.05" length="0.02"/></geometry>
          <material name="black"/>
        </visual>
        <collision>
          <origin xyz="0 0 0" rpy="1.5707 0 0"/>
          <geometry><cylinder radius="0.05" length="0.02"/></geometry>
        </collision>
        <inertial>
          <mass value="0.2"/>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <inertia ixx="0.0002" ixy="0" ixz="0" iyy="0.0002" iyz="0" izz="0.0004"/>
        </inertial>
      </link>
      <joint name="right_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="right_wheel_link"/>
        <origin xyz="0 -0.11 -0.05" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
      </joint>

      <!-- LiDAR SENSOR -->
      <link name="laser_link">
        <visual>
          <geometry><cylinder radius="0.03" length="0.04"/></geometry>
          <material name="red"><color rgba="1 0 0 1"/></material>
        </visual>
        <collision><geometry><cylinder radius="0.03" length="0.04"/></geometry></collision>
        <inertial>
          <mass value="0.1"/>
          <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
        </inertial>
      </link>
      <joint name="laser_joint" type="fixed">
        <parent link="base_link"/>
        <child link="laser_link"/>
        <origin xyz="0.15 0 0.05" rpy="0 0 0"/>
      </joint>

      <!-- Gazebo reference for materials -->
      <gazebo reference="base_link">
        <material>Gazebo/Blue</material>
      </gazebo>
      <gazebo reference="caster_wheel_link">
        <material>Gazebo/Black</material>
      </gazebo>
      <gazebo reference="left_wheel_link">
        <material>Gazebo/Black</material>
      </gazebo>
      <gazebo reference="right_wheel_link">
        <material>Gazebo/Black</material>
      </gazebo>
      <gazebo reference="laser_link">
        <material>Gazebo/Red</material>
      </gazebo>

      <!-- Gazebo differential drive plugin -->
      <gazebo>
        <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
          <ros>
            <namespace>/</namespace>
          </ros>
          <left_joint>left_wheel_joint</left_joint>
          <right_joint>right_wheel_joint</right_joint>
          <wheel_separation>0.22</wheel_separation>
          <wheel_radius>0.05</wheel_radius>
          <publish_wheel_tf>true</publish_wheel_tf>
          <publish_odom>true</publish_odom>
          <publish_wheel_speed>true</publish_wheel_speed>
          <odometry_frame>odom</odometry_frame>
          <robot_base_frame>base_link</robot_base_frame>
          <alwaysOn>true</alwaysOn>
          <updateRate>100</updateRate>
          <legacyMode>false</legacyMode>
          <cmd_vel_topic>cmd_vel</cmd_vel_topic>
        </plugin>
      </gazebo>

      <!-- Gazebo LiDAR plugin -->
      <gazebo reference="laser_link">
        <sensor name="laser_sensor" type="ray">
          <pose>0 0 0 0 0 0</pose>
          <visualize>true</visualize>
          <update_rate>10</update_rate>
          <ray>
            <scan>
              <horizontal>
                <samples>360</samples>
                <resolution>1</resolution>
                <min_angle>-3.14</min_angle>
                <max_angle>3.14</max_angle>
              </horizontal>
            </scan>
            <range>
              <min>0.1</min>
              <max>5.0</max>
              <resolution>0.01</resolution>
            </range>
          </ray>
          <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_laser.so">
            <ros>
              <argument>~/out</argument>
              <namespace>/</namespace>
            </ros>
            <topicName>scan</topicName>
            <frameName>laser_link</frameName>
          </plugin>
        </sensor>
      </gazebo>

    </robot>
    ```

## 3. Create a Simple Gazebo World

We'll use a simple world with a ground plane and a wall to test the LiDAR.

1.  Create `my_mobile_robot/worlds/obstacle_world.world`:
    ```xml
    <?xml version="1.0" ?>
    <sdf version="1.7">
      <world name="obstacle_world">
        <include>
          <uri>model://sun</uri>
        </include>
        <include>
          <uri>model://ground_plane</uri>
        </include>

        <!-- A simple wall model -->
        <model name="wall">
          <pose>2 0 0.5 0 0 0</pose>
          <static>true</static>
          <link name="link">
            <collision name="collision">
              <geometry><box><size>0.1 2 1</size></box></geometry>
            </collision>
            <visual name="visual">
              <geometry><box><size>0.1 2 1</size></box></geometry>
              <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
            </visual>
          </link>
        </model>
      </world>
    </sdf>
    ```

## 4. Create a Launch File to Spawn the Robot

This launch file will start Gazebo with our world and spawn the mobile robot.

1.  Create `my_mobile_robot/launch/mobile_robot_world.launch.py`:
    ```python
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node

    def generate_launch_description():
        # Get path to your URDF file
        urdf_file_path = os.path.join(
            get_package_share_directory('my_mobile_robot'),
            'urdf',
            'mobile_robot.urdf')

        # Get path to your world file
        world_file_path = os.path.join(
            get_package_share_directory('my_mobile_robot'),
            'worlds',
            'obstacle_world.world')

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

        # Node for spawning the robot in Gazebo
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                       '-entity', 'my_mobile_robot',
                       '-x', '0.0', '-y', '0.0', '-z', '0.1'], # Slightly above ground
            output='screen'
        )

        return LaunchDescription([
            gazebo,
            rsp_node,
            spawn_entity,
        ])
    ```

## 5. Write the LiDAR Obstacle Detector Node

This Python node will read LiDAR data and publish velocity commands.

1.  Create `my_mobile_robot/python/my_mobile_robot/li_detector.py`:
    ```python
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import Twist

    class LiDARDetector(Node):
        def __init__(self):
            super().__init__('li_detector')
            self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
            self.subscription = self.create_subscription(
                LaserScan,
                'scan',
                self.scan_callback,
                10)
            self.get_logger().info('LiDAR Detector node has been started.')
            self.cmd_vel_msg = Twist()
            self.obstacle_detected = False

        def scan_callback(self, msg):
            # Find the minimum range in the front sector (e.g., -30 to +30 degrees)
            # Assuming 360 samples, 0 is directly front, 180 is directly back
            # For simplicity, check 30 degrees left and right of center
            front_angles_min_idx = int(len(msg.ranges) * (-30 / 360)) % len(msg.ranges)
            front_angles_max_idx = int(len(msg.ranges) * (30 / 360)) % len(msg.ranges)

            # Check for obstacles in a simple front window
            min_front_range = float('inf')
            for i in range(front_angles_min_idx, front_angles_max_idx + 1):
                if msg.ranges[i] < min_front_range:
                    min_front_range = msg.ranges[i]

            # Define a threshold for obstacle detection
            if min_front_range < 0.5: # Obstacle within 0.5 meters
                self.obstacle_detected = True
            else:
                self.obstacle_detected = False

            self.publish_cmd_vel()

        def publish_cmd_vel(self):
            if self.obstacle_detected:
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.angular.z = 0.0
                self.get_logger().info("Obstacle detected! Stopping.")
            else:
                self.cmd_vel_msg.linear.x = 0.2 # Move forward
                self.cmd_vel_msg.angular.z = 0.0
                self.get_logger().info("No obstacle. Moving forward.")

            self.publisher_.publish(self.cmd_vel_msg)

    def main(args=None):
        rclpy.init(args=args)
        node = LiDARDetector()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

## 6. Update `CMakeLists.txt` and `setup.py`

1.  Open `my_mobile_robot/CMakeLists.txt` and add the new directories for installation:
    ```cmake
    # ... other installs
    install(DIRECTORY
      urdf
      launch
      worlds
      python/my_mobile_robot # This installs the python module
      DESTINATION share/${PROJECT_NAME}
    )
    ```

2.  Open `my_mobile_robot/setup.py` and add the `li_detector` node as an entry point:
    ```python
    entry_points={
        'console_scripts': [
            'li_detector = my_mobile_robot.li_detector:main', # Add this line
        ],
    },
    ```

## 7. Build and Run the Simulation

1.  Navigate to your workspace root (`cd ~/ros2_ws`) and build the package:
    ```bash
    colcon build --packages-select my_mobile_robot
    ```
2.  Source the overlay:
    ```bash
    . install/setup.bash
    ```
3.  Launch the Gazebo world with the robot:
    ```bash
    ros2 launch my_mobile_robot mobile_robot_world.launch.py
    ```
    This will open Gazebo with your mobile robot and a wall.

4.  **Open a NEW terminal** (and source the overlay again).
5.  Run your LiDAR detector node:
    ```bash
    ros2 run my_mobile_robot li_detector
    ```

## Expected Behavior

-   In Gazebo, your robot should start moving forward.
-   When the robot gets close to the wall (within 0.5 meters), it should stop.
-   If you manually move the robot away from the wall (e.g., using the Gazebo GUI controls), it should start moving forward again.
-   You can optionally open RViz2 (`rviz2`) and add a `LaserScan` display subscribed to the `/scan` topic to visualize the LiDAR data.

**Congratulations! Your robot is now equipped with perception and basic reactive behavior in a simulated environment!**

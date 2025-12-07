# Chapter 6: Sensors and Actuators in Gazebo

## Introduction

In the previous chapter, you successfully spawned your URDF arm into Gazebo and observed its basic physics. This was a crucial step, but a robot is more than just a collection of inert links and joints. To interact with its environment, a robot needs **sensors** to perceive and **actuators** to move.

This chapter delves into how to equip your simulated robots in Gazebo with virtual sensors like LiDARs, cameras, and IMUs, and how to control their joints and movement through ROS 2 interfaces. By the end, your simulated robot will be able to 'see', 'feel', and 'move' within its digital world, just like its real-world counterparts.

## Learning Outcomes

By the end of this chapter, you will be able to:

-   Add various types of sensors (e.g., LiDAR, camera, IMU) to a URDF/SDF model for use in Gazebo.
-   Understand the ROS 2 message types associated with common sensors.
-   Use Gazebo plugins to expose sensor data and actuator commands via ROS 2 topics.
-   Write a simple ROS 2 Python node to process sensor data (e.g., read LiDAR scans).
-   Write a ROS 2 Python node to send commands to control a simulated robot's movement.

## Key Concepts

### Sensor Plugins

Gazebo provides a rich set of plugins that mimic real-world sensors. These plugins are added to your URDF/SDF model and tell Gazebo to simulate sensor behavior and publish their readings to ROS 2 topics.

Common Gazebo ROS 2 sensor plugins:

-   **`libgazebo_ros_laser.so` (LiDAR)**: Simulates a 2D or 3D laser scanner. Publishes `sensor_msgs/LaserScan` or `sensor_msgs/PointCloud2` messages. Crucial for mapping and navigation.
-   **`libgazebo_ros_camera.so` (Camera)**: Simulates a monocular, stereo, or depth camera. Publishes `sensor_msgs/Image` (for color/grayscale) and `sensor_msgs/PointCloud2` (for depth cameras). Essential for object detection, visual SLAM, and scene understanding.
-   **`libgazebo_ros_imu.so` (IMU - Inertial Measurement Unit)**: Simulates an accelerometer, gyroscope, and magnetometer. Publishes `sensor_msgs/Imu` messages. Important for robot localization and attitude estimation.
-   **`libgazebo_ros_force_torque.so` (Force/Torque Sensor)**: Simulates a sensor at a joint to measure forces and torques. Publishes `geometry_msgs/WrenchStamped`. Useful for manipulation tasks.

When adding a sensor, you typically define its `link` (where it's mounted), its `joint` (if it moves), and specific parameters like field of view, resolution, update rate, and the ROS 2 topic it will publish to.

### Actuator Control Plugins

Just as sensors perceive, actuators execute. Gazebo ROS 2 also provides plugins to control your simulated robot's joints and movement via ROS 2 topics.

-   **`libgazebo_ros_control.so`**: This is a powerful plugin that connects Gazebo's physics engine to the `ros2_control` framework. It allows you to use sophisticated ROS 2 controllers (e.g., position, velocity, effort controllers) to command your robot's joints.
-   **`libgazebo_ros_diff_drive.so` (Differential Drive)**: Specifically for wheeled robots, this plugin takes `geometry_msgs/Twist` messages (linear and angular velocity commands) from a ROS 2 topic (e.g., `/cmd_vel`) and applies the appropriate forces to the robot's wheels to make it move.

To use these, you modify your URDF/SDF to include the plugin and define how it interacts with the robot's joints.

## Important Definitions

-   **Sensor**: A device that detects and responds to some type of input from the physical environment.
-   **Actuator**: A component of a machine that is responsible for moving and controlling a mechanism or system.
-   **LiDAR (Light Detection and Ranging)**: A method for measuring distances by illuminating the target with laser light and measuring the reflection with a sensor.
-   **IMU (Inertial Measurement Unit)**: An electronic device that measures and reports a body's specific force, angular rate, and sometimes the orientation of the body, using a combination of accelerometers and gyroscopes.
-   **`ros2_control`**: A generic and flexible control framework for ROS 2 that provides a standardized way to define robot controllers.
-   **Differential Drive**: A common drive system for mobile robots where two (or more) wheels are independently driven.

## Real-life Robotics Examples

-   **Autonomous Mobile Robot**: A robot using a LiDAR to build a map of its environment and avoid obstacles, a camera for visual navigation, and differential drive actuators to move around. All these components are integrated via Gazebo ROS 2 plugins.
-   **Humanoid Robot**: A humanoid might use IMUs to maintain balance, force/torque sensors in its hands to grasp objects gently, and joint position controllers to execute precise arm movements. Each of these would have a corresponding Gazebo plugin.

## Required Python/ROS 2 Skills

-   Familiarity with URDF/SDF (from Chapter 4).
-   Knowledge of ROS 2 topics and messages (from Chapter 2).
-   Basic Python scripting.

## Hands-on Lab Summary

In this lab, you will take a simple mobile robot model and enhance it with sensors and actuators in Gazebo. You will:
1.  Extend the URDF of a basic mobile robot to include a LiDAR sensor and define it as a differential drive robot.
2.  Add the necessary Gazebo ROS 2 plugins for the LiDAR (`libgazebo_ros_laser.so`) and the differential drive (`libgazebo_ros_diff_drive.so`).
3.  Launch this enhanced robot in Gazebo.
4.  Use RViz2 to visualize the LiDAR sensor data (points representing detected obstacles).
5.  Write a simple ROS 2 Python node that subscribes to the LiDAR topic and publishes velocity commands to make the robot drive forward, demonstrating basic reactive behavior.

*(The detailed, step-by-step lab instructions will be in a separate file in the `labs/` directory.)*

## Summary

You have now learned how to equip your simulated robots with perception capabilities using various Gazebo ROS 2 sensor plugins, and how to control their movement using actuator plugins. Your robot can now 'see' and 'move' within its virtual environment, reacting to the simulated world around it. This lays the groundwork for more advanced topics like navigation, mapping, and interaction. In the next chapter, we will explore another powerful simulation environment, Unity, for even higher fidelity visuals and richer interactions.

# Lab 9: Simulating Visual SLAM in Isaac Sim

## Objective
This lab will guide you through setting up a camera sensor in Isaac Sim to generate realistic image data and then integrating this data with an external ROS 2 Visual SLAM (VSLAM) package. You will learn how to feed simulated camera streams into a VSLAM algorithm to build a map of the environment and localize your robot.

**Note**: Implementing a full VSLAM algorithm from scratch is beyond the scope of this lab. We will focus on configuring Isaac Sim to provide the necessary data and interfacing with existing, well-established VSLAM ROS 2 packages.

## Prerequisite Skills
-   A working ROS 2 Humble installation.
-   Completion of Lab 8 (familiarity with Isaac Sim and ROS 2 integration).
-   Basic understanding of camera intrinsics and extrinsics (helpful).

---

## Part 1: Setting Up the Isaac Sim Environment with a Camera Robot

### 1. Launch Isaac Sim

1.  Open the NVIDIA Omniverse Launcher and launch Isaac Sim.
2.  Open a new stage (`File > New Stage`).

### 2. Add a Mobile Robot with a Camera

For VSLAM, we need a mobile robot with a camera. Isaac Sim provides various pre-built assets.

1.  **Add a simple mobile robot**:
    -   In the "Content" window, navigate to `Isaac > Robots > ROS_Mobile_Robot` or `Isaac > Robots > TurtleBot3`. Drag and drop one onto your stage.
2.  **Add a camera sensor**: Most pre-built robots in Isaac Sim already include cameras. If not, you can add a camera to a link of your robot:
    -   Right-click on the robot's primary link (e.g., `base_link` if using a simple mobile base) in the "Stage" window.
    -   Go to `Add > Sensor > Camera`.
    -   In the "Property" window for the new camera, configure its parameters:
        -   Adjust `Resolution` (e.g., `640x480`).
        -   Set `Focal Length` and `Horizontal Aperture`.
        -   Ensure `Render Type` is set to `Pinhole` or similar.
3.  **Enable ROS 2 Camera Publisher**:
    -   Select your camera in the "Stage" window.
    -   In the "Property" window, go to `Add > Isaac Ros Bridge > ROS2 Camera`.
    -   Configure the `topic_name` (e.g., `/camera/rgb/image_raw`) and `frame_id` (e.g., `camera_link`).
    -   Ensure the `image_width` and `image_height` match your camera's resolution.
4.  **Add Odometry Publisher (Optional but Recommended for VSLAM)**:
    -   Many VSLAM algorithms benefit from robot odometry (wheel encoder data) as an input or for initialization.
    -   If your chosen robot has wheel encoders, ensure an odometry publisher extension is enabled (e.g., `omni.isaac.ros2_odom`). This typically publishes to `/odom`.

### 3. Set Up an Environment for Mapping

1.  Add various static objects to your scene from the "Content" window (`Isaac > Props` or `Isaac > Environments`) to create a rich environment for mapping. Think of objects like cubes, cylinders, tables, etc., that can serve as visual features.
2.  Start the Isaac Sim simulation by pressing the "Play" button.

## Part 2: Integrating with an External ROS 2 VSLAM Package

### 1. Install a ROS 2 VSLAM Package

Choose a VSLAM package compatible with ROS 2 Humble. Popular options include:

-   **`rtabmap_ros`**: An open-source graph-based SLAM approach that can use various sensor inputs (RGB-D, Stereo, LiDAR).
    -   Installation: `sudo apt install ros-humble-rtabmap-ros`
-   **ORB-SLAM3**: A versatile and accurate open-source VSLAM system. Requires building from source.
    -   Refer to: [https://github.com/UZ-SLAMLab/ORB_SLAM3_ROS2](https://github.com/UZ-SLAMLab/ORB_SLAM3_ROS2) (Follow build instructions carefully).

For this lab, we'll assume you install `rtabmap_ros` as it's easier to get started with.

### 2. Configure the VSLAM Package

The VSLAM package needs to know where to find your camera and odometry data.

1.  **Create a launch file** in your ROS 2 workspace (e.g., in `my_first_package/launch` or a new package for VSLAM config).
2.  This launch file will typically:
    -   Start the `rtabmap` node(s).
    -   Remap topics if necessary (e.g., ensure `rtabmap` subscribes to `/camera/rgb/image_raw` and `/odom`).
    -   Set parameters for `rtabmap` (e.g., feature detector, map update rates).
    -   Optionally launch `RViz2` to visualize the map and robot pose.

    A simplified example structure for `rtabmap.launch.py`:
    ```python
    import os
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),

            Node(
                package='rtabmap_ros', executable='rtabmap', output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'frame_id': 'base_link', # Or your robot's base frame
                    'subscribe_depth': False, # True if using depth camera
                    'subscribe_rgb': True,
                    'subscribe_stereo': False,
                    'approx_sync': True,
                    'Rtabmap/DetectionRate': '5.0', # Process at 5 Hz
                }],
                remappings=[
                    ('/rgb/image', '/camera/rgb/image_raw'),
                    ('/rgb/camera_info', '/camera/rgb/camera_info'),
                    ('/odom', '/odom') # Assuming your robot publishes /odom
                ],
                arguments=['-d'] # Delete database on start
            ),
            Node(
                package='rtabmap_ros', executable='rtabmapviz', output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'frame_id': 'base_link',
                    'subscribe_depth': False,
                    'subscribe_rgb': True,
                    'subscribe_stereo': False,
                    'approx_sync': True,
                }],
                remappings=[
                    ('/rgb/image', '/camera/rgb/image_raw'),
                    ('/rgb/camera_info', '/camera/rgb/camera_info'),
                    ('/odom', '/odom')
                ],
            )
        ])
    ```
    *(Note: You will need to customize this launch file heavily based on your specific robot's topics and frame IDs, and the VSLAM algorithm's requirements.)*

## Part 3: Run the VSLAM Simulation

1.  **Launch Isaac Sim**: Start Isaac Sim and load your robot with the camera and environment. Press "Play" to start the simulation.
2.  **Launch the VSLAM package**:
    -   In a new terminal (after sourcing your ROS 2 workspace), run your VSLAM launch file:
        ```bash
        ros2 launch my_vslam_config rtabmap.launch.py
        ```
3.  **Drive the Robot (Manually or Autonomously)**:
    -   You can manually drive your robot around in Isaac Sim (e.g., using a gamepad, a keyboard teleop node, or simply by moving its position in the UI if physics is off).
    -   As the robot moves, `rtabmap_ros` will consume the camera images (and odometry if available) and start building a map.
4.  **Visualize the Map**:
    -   If `rtabmapviz` or `RViz2` was launched, you should see a 3D map forming as your robot explores the environment. You will also see the estimated trajectory of your robot.

## Expected Behavior

-   As your robot moves in Isaac Sim, an external ROS 2 VSLAM node processes the simulated camera feed.
-   A 3D map of the explored environment is incrementally built and visualized.
-   The VSLAM algorithm accurately estimates the robot's pose within this map.

**Congratulations! You have successfully integrated a simulated robot in Isaac Sim with an external ROS 2 VSLAM pipeline, demonstrating how to build intelligent perception systems!**

# Lab 13: Capstone Project - End-to-End VLA Humanoid in Isaac Sim

## Objective
This capstone lab challenges you to integrate all the knowledge and skills acquired throughout the book into a single, functional Vision-Language-Action (VLA) system. You will orchestrate speech-to-text, LLM-based planning, object detection, autonomous navigation, and manipulation for a humanoid robot in NVIDIA Isaac Sim, culminating in a robot that can understand voice commands and execute complex tasks.

**Note**: This is an advanced integration lab. While we provide a comprehensive outline, each component (especially external AI models and complex robot configurations) will require careful setup and adherence to their respective documentation.

## Prerequisite Skills
-   Completion of all previous labs (ROS 2 basics, Gazebo, Unity, Isaac Sim, VSLAM, Nav2, LLM planning, VLA pipeline).
-   Proficiency in Python and ROS 2.
-   Access to a powerful system with an NVIDIA GPU and Isaac Sim installed.
-   API keys for LLM and STT services.

---

## Part 1: Isaac Sim Setup - Humanoid Robot and Environment

### 1. Launch Isaac Sim

1.  Open the NVIDIA Omniverse Launcher and launch Isaac Sim.
2.  Open a new stage (`File > New Stage`).

### 2. Add a Humanoid Robot

1.  **Choose a Humanoid Model**: In the "Content" window, navigate to `Isaac > Robots > Humanoids` (if available) or select a complex mobile manipulator like the `Franka_Robot` mounted on a mobile base (e.g., `Jackal`). For this lab, a mobile manipulator (like a Franka arm on a Jackal base) offers a good balance of complexity and built-in ROS 2 support.
    -   Drag and drop your chosen robot (e.g., `jackal_franka.usd` if available) onto the stage.
2.  **Populate the Environment**:
    -   Add a complex indoor environment (e.g., `omniverse://localhost/NVIDIA/Assets/Isaac/2023.1/Isaac/Environments/Nav/Nav_Warehouse.usd`).
    -   Add various objects for manipulation and detection from `Isaac > Props` (e.g., cubes, cylinders, tools) to the environment, placing them on tables or the floor.

### 3. Configure ROS 2 Extensions

Ensure all necessary ROS 2 extensions are enabled in Isaac Sim (`Window > Extensions`):
-   `omni.isaac.ros2_bridge`
-   `omni.isaac.ros2_imu`
-   `omni.isaac.ros2_laser_scan`
-   `omni.isaac.ros2_tf_broadcaster`
-   `omni.isaac.ros2_nav2` (if using Nav2)
-   `omni.isaac.ros2_control` (for precise joint control)
-   Any robot-specific extensions (e.g., `omni.isaac.franka`).

## Part 2: External ROS 2 Nodes and AI Services

You will need several external ROS 2 packages and AI services running alongside Isaac Sim.

### 1. Speech-to-Text (STT) Service

You can either:
-   **Use a Microhone with OpenAI Whisper API**: (Recommended for real-time interaction)
    -   Set up a Python script that records audio, sends it to the Whisper API, and publishes the transcribed text to a ROS 2 topic (e.g., `/voice_command`).
-   **Use a simulated text publisher**: For debugging, publish text directly to `/voice_command`.

### 2. Object Detection Node

This node will provide the LLM with information about objects in the scene.

1.  **Choose an Object Detector**: Select a ROS 2 compatible object detection package (e.g., a YOLO-based detector, or a simple color/shape detector if not using a pre-trained model).
2.  **Integrate with Isaac Sim Camera**: Configure the detector to subscribe to the robot's camera image topic (e.g., `/camera/rgb/image_raw`) published by Isaac Sim.
3.  **Publish Detected Objects**: The node should publish a custom ROS 2 message containing `object_name`, `bounding_box`, and `3D_pose` for each detected object to a topic (e.g., `/detected_objects`).

### 3. ROS 2 Navigation Stack (Nav2)

1.  **Map the Environment**: If your chosen Isaac Sim environment doesn't have a pre-built map, generate one using SLAM (from Lab 9) by driving your robot around. Save the map.
2.  **Launch Nav2**: Configure and launch the full Nav2 stack (AMCL, global/local planners, behavior tree) using a launch file that loads your map and is tailored for your humanoid robot (from Lab 10). Ensure it subscribes to the correct odometry, LiDAR, and TF topics from Isaac Sim.

### 4. ROS 2 Manipulation Controller

For picking and placing, you'll need to control the robot's arm.

1.  **`ros2_control`**: Leverage Isaac Sim's `ros2_control` integration. You'll likely use position or velocity controllers for the arm joints.
2.  **Inverse Kinematics (IK)**: You'll need an IK solver (e.g., `moveit_kinematics` or a custom Python IK solver) to translate desired end-effector poses (e.g., "reach for the apple") into joint angles.
3.  **Motion Planning**: For complex arm movements, `MoveIt 2` is the standard ROS 2 solution. You would integrate `MoveIt 2` to generate collision-free trajectories for the arm.

### 5. Text-to-Speech (TTS) Service

Set up a Python script that subscribes to a ROS 2 topic (e.g., `/robot_responses`), converts incoming text messages into audio using a TTS API (e.g., Google Text-to-Speech, AWS Polly, or a local library), and plays the audio.

## Part 3: The Central VLA Orchestration Node (Python)

This will be the heart of your capstone project, building upon `vla_system.py` from Lab 12.

1.  **Subscribe to Voice Commands**: Subscribe to `/voice_command` (from STT).
2.  **Subscribe to Object Detections**: Subscribe to `/detected_objects`.
3.  **LLM Integration**:
    -   When a new voice command arrives, format a comprehensive prompt for your LLM (using context from `/detected_objects`).
    -   Send the prompt to the LLM API (e.g., OpenAI, Gemini, Claude).
    -   Parse the LLM's JSON plan into a sequence of executable actions.
4.  **Action Executive**:
    -   Implement logic to sequentially execute the LLM's plan.
    -   **NAVIGATE_TO**: Publish to Nav2's `/goal_pose` topic. Wait for Nav2 to report completion.
    -   **SCAN_FOR_OBJECTS**: Trigger the object detection node (if not continuously running) and wait for new detections.
    -   **PICK_UP / PLACE_DOWN**:
        -   Use object detection data to find the target object's 3D pose.
        -   Call the IK solver to get joint angles for reaching.
        -   Send joint commands via `ros2_control` to the robot's arm.
        -   Control a gripper (open/close).
    -   **REPORT_STATUS**: Publish text to `/robot_responses` (for TTS).
5.  **Error Handling**: Implement basic error handling (e.g., if Nav2 fails, if object not found).

## Part 4: Run the Capstone Project

1.  **Launch Isaac Sim**: Start Isaac Sim with your humanoid robot and populated environment. Press "Play".
2.  **Launch External ROS 2 Nodes**:
    -   Launch your STT script (if using microphone).
    -   Launch your Object Detection Node.
    -   Launch the Nav2 Stack.
    -   Launch your Manipulation Controller (if separate).
    -   Launch your TTS script.
3.  **Launch Your VLA Orchestration Node**:
    ```bash
    ros2 run your_vla_package vla_orchestrator
    ```
4.  **Issue Voice Commands**: Speak commands to your robot (e.g., "Robot, find the red cube, go to it, pick it up, and bring it to me").

## Expected Behavior

-   The humanoid robot in Isaac Sim will respond to your voice commands.
-   It will navigate autonomously, visually identify objects, manipulate them (pick and place), and provide spoken feedback on its progress and completion.
-   This will be a dynamic, interactive demonstration of an intelligent robot system.

**Congratulations! You have completed the Capstone Project and built a fully integrated Vision-Language-Action humanoid robot system!** This is the ultimate demonstration of your mastery of Physical AI and Robotics.

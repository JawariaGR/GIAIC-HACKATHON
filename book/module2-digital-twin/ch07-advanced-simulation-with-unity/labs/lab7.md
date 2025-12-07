# Lab 7: Controlling a Robot Model in Unity with ROS 2

## Objective
This lab will guide you through the process of setting up a Unity project for robotics simulation and establishing two-way communication with ROS 2. You will learn how to import a robot model, configure it for ROS 2 control, and use a simple Python node to send commands to your Unity robot.

**Note**: This lab involves setting up a Unity project and installing external packages. While we provide detailed steps, some parts will refer to external documentation and require manual interaction with the Unity editor.

## Prerequisite Skills
-   A working ROS 2 Humble installation.
-   Basic understanding of Python scripting.
-   Access to a computer capable of running Unity (Windows, macOS, or Linux).

---

## Part 1: Setting Up Your Unity Project

### 1. Install Unity Hub and Unity Editor

1.  Download and install **Unity Hub** from the official Unity website: [https://unity.com/download](https://unity.com/download)
2.  Install a **Unity Editor** version (e.g., Unity 2022 LTS or newer) through Unity Hub.

### 2. Create a New Unity Project

1.  Open **Unity Hub**.
2.  Click "New Project" and select a 3D template (e.g., "3D Core").
3.  Choose a project name (e.g., `RosUnityArmControl`) and location.
4.  Click "Create Project".

### 3. Install the ROS-Unity Integration Package

The ROS-Unity Integration package bridges Unity with ROS 2.

1.  **Download the latest release of `ROS-TCP-Endpoint` and `ROS-TCP-Connector`**:
    Go to the GitHub releases page for `Unity-Technologies/ROS-TCP-Endpoint` and download the `.unitypackage` files: [https://github.com/Unity-Technologies/ROS-TCP-Endpoint/releases](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/releases)
    
2.  **Import into Unity**:
    -   In your Unity project, go to `Assets > Import Package > Custom Package...`.
    -   Select the downloaded `ROS-TCP-Endpoint.unitypackage` and `ROS-TCP-Connector.unitypackage` files. Import all items.

### 4. Configure ROS 2 IP Address

The ROS-Unity Integration needs to know where your ROS 2 system is running.

1.  In Unity, navigate to `Robotics > ROS TCP Connector > ROSConnection`.
2.  In the Inspector window, find the `ROS IP Address` field.
3.  Enter the IP address of the machine running your ROS 2 environment (e.g., your Ubuntu VM's IP address). If running on the same machine, use `127.0.0.1`.

## Part 2: Importing and Configuring Your Robot Model

For this lab, you can either:
    A. Convert your `my_arm.urdf` to an FBX/OBJ model and import it.
    B. Use a simpler pre-made robot model from the Unity Asset Store or provided with the course materials.
We will assume you have a simple 2-DOF arm model imported (e.g., as a `GameObject` hierarchy where each joint is a child).

### 1. Import Your Robot Model

1.  Drag and drop your 3D model file (e.g., `.fbx`, `.obj`) into the `Assets` folder in your Unity Project window.
2.  Drag the imported model from the `Assets` folder into your scene hierarchy.
3.  Position the robot appropriately in your scene.

### 2. Configure Joints for ROS 2 Control

For each joint you want to control:

1.  Select the `GameObject` representing the joint in your Hierarchy (e.g., `arm_link_1`, `arm_link_2`).
2.  In the Inspector window, click "Add Component" and search for `Joint State Publisher` (from the ROS-Unity Integration package). Add it.
3.  Add another component: `Joint State Subscriber`.
4.  In the `Joint State Subscriber` component:
    -   Set the `Topic Name` (e.g., `/joint_commands`).
    -   Ensure the `Joint Name` matches the name of your joint from the URDF (e.g., `joint1`, `joint2`).
    -   Set the `Joint Type` (e.g., `Revolute`).

Repeat this for all controllable joints in your robot.

## Part 3: Writing a ROS 2 Python Controller

Now, let's create a Python node in your ROS 2 workspace that publishes joint commands to Unity.

1.  Navigate to your `~/ros2_ws/src/my_first_package/my_first_package/` directory.
2.  Create a new file named `unity_arm_controller.py`:
    ```python
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    import time

    class UnityArmController(Node):
        def __init__(self):
            super().__init__('unity_arm_controller')
            self.publisher_ = self.create_publisher(JointState, '/joint_commands', 10)
            self.timer = self.create_timer(1.0, self.timer_callback) # Publish every second
            self.joint_names = ['joint1', 'joint2'] # Replace with your joint names
            self.joint_positions = [0.0, 0.0]
            self.get_logger().info('Unity Arm Controller node started.')

        def timer_callback(self):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            
            # Simple alternating movement
            self.joint_positions[0] = (self.joint_positions[0] + 0.1) % 1.57 # Rotate joint1
            self.joint_positions[1] = (self.joint_positions[1] + 0.05) % 0.785 # Rotate joint2

            msg.position = self.joint_positions
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing JointState: {msg.position}')

    def main(args=None):
        rclpy.init(args=args)
        node = UnityArmController()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

3.  Add this new node to your `my_first_package/setup.py`'s `console_scripts` entry points:
    ```python
    'console_scripts': [
        # ... other nodes ...
        'unity_arm_controller = my_first_package.unity_arm_controller:main',
    ],
    ```

## Part 4: Run the Simulation

1.  **Build your ROS 2 workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_first_package
    . install/setup.bash
    ```

2.  **Start your Unity application**:
    -   In the Unity Editor, click the "Play" button. The scene will start.

3.  **Run your ROS 2 controller**:
    -   In a new terminal (after sourcing your workspace), run:
        ```bash
        ros2 run my_first_package unity_arm_controller
        ```

## Expected Behavior

-   In the Unity Editor's game view, you should see your robot arm's joints rotating according to the commands published by your ROS 2 Python node.
-   The ROS 2 terminal will show "Publishing JointState" messages.

**Congratulations! You have established communication between ROS 2 and Unity, controlling a robot in a high-fidelity simulation environment.** This integration allows you to leverage Unity's powerful visualization and interaction capabilities with ROS 2's robust robotics ecosystem.

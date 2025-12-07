# Lab 8: Controlling a Franka Emika Robot in Isaac Sim with ROS 2

## Objective
In this lab, you will get hands-on with NVIDIA Isaac Sim. You will learn to launch the simulator, load a pre-built robotic arm (the Franka Emika Panda), and establish ROS 2 control over its joints using Python nodes.

**Note**: Isaac Sim is a powerful, resource-intensive simulator. Ensure your system meets the minimum requirements (NVIDIA GPU, sufficient RAM). This lab will primarily guide you through the Isaac Sim UI and its Python scripting interface, relying on its built-in ROS 2 functionalities.

## Prerequisite Skills
-   A working ROS 2 Humble installation.
-   Basic understanding of Python scripting.
-   Access to a system with a compatible NVIDIA GPU and an Isaac Sim installation.
-   Familiarity with ROS 2 `JointState` messages.

---

## Part 1: Setting Up Isaac Sim

### 1. Install NVIDIA Isaac Sim

1.  Follow the official NVIDIA Isaac Sim documentation for installation. This typically involves:
    -   Installing the **NVIDIA Omniverse Launcher**.
    -   Installing **Cache** and **Nucleus** (local server for USD assets).
    -   Installing the **Isaac Sim** application through the Omniverse Launcher.
2.  Ensure you have properly configured the Isaac Sim environment, including installing its Python environment and ROS 2 bridge extensions.
    -   Refer to: [https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_ros.html](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_ros.html)

### 2. Launch Isaac Sim

1.  Open the NVIDIA Omniverse Launcher.
2.  Launch Isaac Sim. This will open the Isaac Sim application.

## Part 2: Loading the Franka Emika Robot

Isaac Sim comes with many pre-built robot models, including the Franka Emika Panda arm.

1.  **Open a New Stage**: In Isaac Sim, go to `File > New Stage`.
2.  **Add the Franka Robot**:
    -   In the "Content" window (usually on the left), navigate to `Isaac > Robots > Franka`.
    -   Drag and drop `franka_alt_fingers.usd` into your scene's "Stage" window.
3.  **Add a Ground Plane**: For better visualization and physics, add a ground plane.
    -   In the "Content" window, navigate to `Isaac > Environments > Simple`.
    -   Drag and drop `simple_room.usd` or `simple_scene.usd` (or just a basic plane) onto the stage.
4.  **Set Initial Pose**: Select the Franka robot in the "Stage" window. In the "Property" window (usually on the right), adjust its position (`Translate Z`) so it's slightly above the ground plane.

## Part 3: Establishing ROS 2 Control

Isaac Sim has built-in ROS 2 integration. For the Franka arm, this is often pre-configured or easily enabled.

### 1. Enable ROS 2 Extensions

1.  Go to `Window > Extensions`.
2.  Search for and enable the following extensions if they are not already:
    -   `omni.isaac.ros2_bridge`
    -   `omni.isaac.ros2_bridge.ROS2PublishJointState` (for publishing joint states from Isaac Sim)
    -   `omni.isaac.ros2_bridge.ROS2SubscribeJointCommand` (for subscribing to joint commands)
    -   `omni.isaac.franka` (specific to the Franka robot)

### 2. Create a ROS 2 Python Controller

We'll use a simple Python node similar to Lab 7 to send joint commands. This time, we'll send to the appropriate topic for the Franka robot in Isaac Sim.

1.  Navigate to your `~/ros2_ws/src/my_first_package/my_first_package/` directory.
2.  Create a new file named `isaac_franka_controller.py`:
    ```python
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    import time

    class IsaacFrankaController(Node):
        def __init__(self):
            super().__init__('isaac_franka_controller')
            # The topic name for Franka joint commands in Isaac Sim is typically /joint_command
            # or related to ros2_control if that's configured. Check Isaac Sim logs/documentation.
            self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)
            self.timer = self.create_timer(1.0, self.timer_callback) # Publish every second
            
            # Joint names for Franka Emika Panda (standard in Isaac Sim)
            self.joint_names = [
                'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
                'panda_joint5', 'panda_joint6', 'panda_joint7',
                'panda_finger_joint1', 'panda_finger_joint2'
            ]
            self.joint_positions = [0.0] * len(self.joint_names)
            self.get_logger().info('Isaac Franka Controller node started.')

        def timer_callback(self):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            
            # Simple alternating movement for the first few joints
            # Adjust these values based on actual joint limits and desired motion
            self.joint_positions[0] = (self.joint_positions[0] + 0.1) % (3.14 * 2) - 3.14 # Joint 1
            self.joint_positions[1] = (self.joint_positions[1] + 0.05) % 1.0 - 0.5         # Joint 2
            self.joint_positions[2] = (self.joint_positions[2] + 0.02) % (3.14 * 2) - 3.14 # Joint 3

            msg.position = self.joint_positions
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing JointState: {msg.position}')

    def main(args=None):
        rclpy.init(args=args)
        node = IsaacFrankaController()
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
        'isaac_franka_controller = my_first_package.isaac_franka_controller:main',
    ],
    ```

## Part 4: Run the Simulation

1.  **Build your ROS 2 workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_first_package
    . install/setup.bash
    ```

2.  **Start the Isaac Sim simulation**:
    -   In the Isaac Sim application, click the "Play" button (usually at the bottom of the viewport). This starts the physics simulation.

3.  **Run your ROS 2 controller**:
    -   In a new terminal (after sourcing your workspace), run:
        ```bash
        ros2 run my_first_package isaac_franka_controller
        ```

## Expected Behavior

-   In the Isaac Sim viewport, the Franka Emika robot's joints should move according to the commands published by your ROS 2 Python node.
-   The ROS 2 terminal will show "Publishing JointState" messages.
-   Observe the realistic physics and rendering provided by Isaac Sim.

**Congratulations! You have successfully controlled a highly detailed robotic arm in NVIDIA Isaac Sim using ROS 2.** This demonstrates the power of Isaac Sim for developing and testing advanced robotics applications.

# Lab 2: Your First ROS 2 Publisher and Subscriber

## Objective
In this lab, you will build and run your first ROS 2 nodes from scratch using Python. You will create a "talker" node that publishes string messages and a "listener" node that subscribes to them, demonstrating the core publisher-subscriber pattern.

## Prerequisite Skills
-   A working ROS 2 Humble installation (from Lab 1).
-   Basic knowledge of Python syntax.
-   Familiarity with a text editor (like VS Code).

---

## 1. Create a ROS 2 Package

First, we need a package to house our new nodes. A ROS 2 package is just a directory with a specific file structure and a `package.xml` file that contains metadata.

1.  Navigate to the `src` directory of your ROS 2 workspace (created in Lab 1):
    ```bash
    cd ~/ros2_ws/src
    ```

2.  Use the `ros2 pkg create` command to create a new package. We'll name it `my_first_package`.
    ```bash
    ros2 pkg create --build-type ament_python --license Apache-2.0 my_first_package --dependencies rclpy std_msgs
    ```
    -   `--build-type ament_python`: Specifies we are creating a Python package.
    -   `--dependencies rclpy std_msgs`: Automatically adds dependencies on the Python client library (`rclpy`) and the package containing standard message types (`std_msgs`).

You should now have a new directory `~/ros2_ws/src/my_first_package`.

## 2. Create the Publisher Node (Talker)

Now, let's write the Python code for our publisher node.

1.  Inside your `my_first_package` directory, create a new file named `talker.py`.
    ```bash
    # For example, using VS Code:
    code ~/ros2_ws/src/my_first_package/my_first_package/talker.py
    ```

2.  Enter the following code into `talker.py`:
    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class TalkerNode(Node):
        def __init__(self):
            super().__init__('talker')
            self.publisher_ = self.create_publisher(String, 'chatter', 10)
            self.timer = self.create_timer(0.5, self.timer_callback) # 0.5 seconds
            self.i = 0
            self.get_logger().info('Talker node has been started.')

        def timer_callback(self):
            msg = String()
            msg.data = f'Hello World: {self.i}'
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.i += 1

    def main(args=None):
        rclpy.init(args=args)
        node = TalkerNode()
        rclpy.spin(node)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

## 3. Create the Subscriber Node (Listener)

Next, we'll create the node that listens for the messages.

1.  Create another new file in the same directory named `listener.py`.
    ```bash
    code ~/ros2_ws/src/my_first_package/my_first_package/listener.py
    ```

2.  Enter the following code into `listener.py`:
    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class ListenerNode(Node):
        def __init__(self):
            super().__init__('listener')
            self.subscription = self.create_subscription(
                String,
                'chatter',
                self.listener_callback,
                10)
            self.get_logger().info('Listener node has been started.')

        def listener_callback(self, msg):
            self.get_logger().info(f'I heard: "{msg.data}"')

    def main(args=None):
        rclpy.init(args=args)
        node = ListenerNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

## 4. Register the Nodes as Entry Points

To make our nodes executable with `ros2 run`, we need to tell the build system about them.

1.  Open the `setup.py` file in the root of your `my_first_package` directory.
2.  Locate the `entry_points` section and add your two nodes, so it looks like this:
    ```python
    entry_points={
        'console_scripts': [
            'talker = my_first_package.talker:main',
            'listener = my_first_package.listener:main',
        ],
    },
    ```

## 5. Build and Run

Now we compile our package and run the nodes.

1.  Navigate to the root of your workspace:
    ```bash
    cd ~/ros2_ws
    ```

2.  Build the package using `colcon`:
    ```bash
    colcon build --packages-select my_first_package
    ```

3.  **In your current terminal**, source the workspace's setup file to make the new executables available:
    ```bash
    . install/setup.bash
    ```

4.  Now, run the talker node:
    ```bash
    ros2 run my_first_package talker
    ```
    You should see the "Publishing: ..." messages appear.

5.  **Open a NEW terminal**. You must source the setup file in this terminal as well.
    ```bash
    cd ~/ros2_ws
    . install/setup.bash
    ```

6.  Run the listener node:
    ```bash
    ros2 run my_first_package listener
    ```
    You should see the "I heard: ..." messages, confirming that your listener is receiving data from your talker!

## Expected Output

Your two terminals should look something like this:

**Terminal 1 (Talker):**
```
[INFO] [talker]: Publishing: "Hello World: 0"
[INFO] [talker]: Publishing: "Hello World: 1"
[INFO] [talker]: Publishing: "Hello World: 2"
...
```

**Terminal 2 (Listener):**
```
[INFO] [listener]: I heard: "Hello World: 0"
[INFO] [listener]: I heard: "Hello World: 1"
[INFO] [listener]: I heard: "Hello World: 2"
...
```

**Congratulations! You have successfully created and run your first ROS 2 nodes.** You have demonstrated the fundamental publisher-subscriber communication pattern.

# Lab 3: Implementing Services and Actions

## Objective
This lab will guide you through implementing both a simple service and a simple action. You will create nodes that act as servers and clients for each communication type, reinforcing your understanding of their differences and use cases. We will continue working in the `my_first_package` you created in Lab 2.

## Prerequisite Skills
-   Completion of Lab 2 (creating a package, publisher, and subscriber).
-   Understanding of Python classes and functions.

---

## Part 1: Creating a Service

Our service will accept a request containing two integers and return a response containing their sum.

### 1. Define the Service (`.srv` file)

First, we need to define the structure of our service's request and response.

1.  Inside your `my_first_package` directory, create a new directory called `srv`.
    ```bash
    mkdir -p ~/ros2_ws/src/my_first_package/srv
    ```

2.  Create a new file inside the `srv` directory named `AddTwoInts.srv`.
    ```bash
    code ~/ros2_ws/src/my_first_package/srv/AddTwoInts.srv
    ```

3.  Add the following content to the file. The part above the `---` is the request, and the part below is the response.
    ```
    int64 a
    int64 b
    ---
    int64 sum
    ```

### 2. Update Package Configuration

We need to tell ROS 2 how to build this new `.srv` file.

1.  Open `package.xml` in `my_first_package` and add these lines to ensure the necessary dependencies are present:
    ```xml
    <build_depend>rosidl_default_generators</build_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```

2.  Open `CMakeLists.txt` and add the following lines to find the package that will generate the Python code from your `.srv` file.
    ```cmake
    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
      "srv/AddTwoInts.srv"
    )
    ```

### 3. Create the Service Server Node

This node will wait for requests and perform the addition.

1.  Create a new file in `my_first_package/my_first_package` named `service_server.py`.
2.  Add the following code:
    ```python
    import rclpy
    from rclpy.node import Node
    from my_first_package.srv import AddTwoInts

    class AddTwoIntsServer(Node):
        def __init__(self):
            super().__init__('add_two_ints_server')
            self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
            self.get_logger().info('Service server has been started.')

        def add_two_ints_callback(self, request, response):
            response.sum = request.a + request.b
            self.get_logger().info(f'Incoming request\n a: {request.a} b: {request.b}')
            self.get_logger().info(f'Returning: [sum: {response.sum}]')
            return response

    def main(args=None):
        rclpy.init(args=args)
        node = AddTwoIntsServer()
        rclpy.spin(node)
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

### 4. Create the Service Client Node

This node will send requests to the server.

1.  Create a file named `service_client.py`.
2.  Add the following code. This client is asynchronous, which is best practice to avoid deadlocks.
    ```python
    import rclpy
    from rclpy.node import Node
    from my_first_package.srv import AddTwoInts

    class AddTwoIntsClient(Node):
        def __init__(self):
            super().__init__('add_two_ints_client')
            self.cli = self.create_client(AddTwoInts, 'add_two_ints')
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.req = AddTwoInts.Request()

        def send_request(self, a, b):
            self.req.a = a
            self.req.b = b
            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            return self.future.result()

    def main(args=None):
        rclpy.init(args=args)
        node = AddTwoIntsClient()
        response = node.send_request(5, 10)
        node.get_logger().info(f'Result of add_two_ints: {response.sum}')
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

### 5. Update `setup.py`

Add the new nodes to your `setup.py` file:
```python
'console_scripts': [
    'talker = my_first_package.talker:main',
    'listener = my_first_package.listener:main',
    'add_server = my_first_package.service_server:main',
    'add_client = my_first_package.service_client:main',
],
```

### 6. Build and Run

1.  Navigate to your workspace root (`cd ~/ros2_ws`) and build:
    ```bash
    colcon build --packages-select my_first_package
    ```
2.  Source the overlay: `. install/setup.bash`
3.  **In one terminal**, run the server:
    ```bash
    ros2 run my_first_package add_server
    ```
4.  **In a new terminal** (after sourcing), run the client:
    ```bash
    ros2 run my_first_package add_client
    ```
    The client will run, print the result (`Result of add_two_ints: 15`), and exit. The server will print that it received a request.

---

## Part 2: Creating an Action (Challenge)

Based on what you learned about services and the concepts from the chapter, try to implement the countdown action yourself!

**Goal**: Create an action that counts down from a given number.
-   **Action Definition**: Goal `int32 seconds`, Feedback `int32 remaining`, Result `string status`.
-   **Action Server**: Receives the goal, and in a loop, prints the remaining time and publishes it as feedback once per second. When finished, it sends a result like "Countdown complete!".
-   **Action Client**: Sends the goal (e.g., 5 seconds) and prints any feedback it receives. When the final result arrives, it prints that too.

This part is more challenging and is meant to test your understanding. The solution will be provided in the book's code repository. Give it a try!

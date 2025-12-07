# Lab 12: Constructing a Basic VLA Pipeline

## Objective
In this lab, you will build a basic Python application that integrates speech-to-text (using OpenAI Whisper), LLM-based task planning (from Lab 11), and a ROS 2 interface for robot control. The goal is to create a system that can take a voice command, process it through the LLM, and then publish the first resulting robot action to a ROS 2 topic.

## Prerequisite Skills
-   A working ROS 2 Humble installation.
-   Completion of Lab 11 (LLM-based task planning).
-   Basic Python programming, including ROS 2 nodes.
-   OpenAI API key (or similar LLM API key).
-   OpenAI Whisper API key (or access to a local Whisper model).

---

## 1. Install OpenAI Python Library

We will use the `openai` library which provides both LLM (GPT) and Whisper (ASR) functionalities.

```bash
pip install openai sounddevice numpy scipy
```

## 2. Set Up Your API Keys

Ensure your `OPENAI_API_KEY` is set as an environment variable (as in Lab 11).
You will also need an `OPENAI_API_KEY` for Whisper. It's the same key if using OpenAI.

## 3. Create the `vla_system.py` Script

This script will combine the components. We'll simulate speech input directly from text for simplicity, but the code structure allows for real audio input.

1.  Create a new Python file (e.g., `vla_system.py`) in your `my_first_package/my_first_package` directory.
2.  Add the following code:
    ```python
    import os
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String # For general commands
    from geometry_msgs.msg import Twist # For navigation commands
    import openai
    import json
    import threading
    import time
    # For simulating speech input - in a real system, you'd use a microphone
    # import sounddevice as sd
    # from scipy.io.wavfile import write as write_wav
    # import numpy as np

    # --- Configuration ---
    openai.api_key = os.getenv("OPENAI_API_KEY")
    LLM_MODEL = "gpt-3.5-turbo" # Or gpt-4, gpt-4o

    ROBOT_CAPABILITIES_PROMPT = """
    You are a robot task planner. Your goal is to break down high-level human commands
    into a sequence of simple, robot-executable actions.

    The robot has the following capabilities (actions):
    - NAVIGATE_TO(location: str)
    - PICK_UP(object: str)
    - PLACE_DOWN(location: str)
    - REPORT_STATUS(message: str)
    - MOVE_FORWARD(distance: float)
    - STOP()
    - SCAN_FOR_OBJECTS()

    Please respond ONLY with a JSON array of these actions. Each action should be a JSON object
    with an "action" key and relevant "params" keys.
    """

    # --- LLM Interaction Function ---
    def get_robot_plan_from_llm(user_command: str, detected_objects: list[str]) -> list[dict]:
        """
        Queries an LLM to generate a sequence of robot actions.
        Includes detected objects in the context.
        """
        object_context = f"Currently detected objects: {', '.join(detected_objects)}." if detected_objects else "No specific objects detected."
        
        full_prompt = f"""
        {ROBOT_CAPABILITIES_PROMPT}

        Current environment context: {object_context}

        Now, for the following human command: "{user_command}"
        Robot plan:
        """

        messages = [
            {"role": "system", "content": "You are a helpful robot assistant."},
            {"role": "user", "content": full_prompt}
        ]

        try:
            response = openai.ChatCompletion.create(
                model=LLM_MODEL,
                messages=messages,
                temperature=0.0 # Keep responses deterministic for planning
            )
            llm_response_content = response.choices[0].message['content'].strip()
            return json.loads(llm_response_content)
        except json.JSONDecodeError as e:
            print(f"Error parsing LLM response JSON: {e}")
            print(f"LLM Response: {llm_response_content}")
            return []
        except Exception as e:
            print(f"An error occurred during LLM API call: {e}")
            return []

    # --- ROS 2 Node for VLA System ---
    class VLASystemNode(Node):
        def __init__(self):
            super().__init__('vla_system_node')
            self.get_logger().info('VLA System Node started.')

            # Publishers for robot commands
            self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
            self.status_publisher = self.create_publisher(String, 'robot_status', 10)
            
            self.current_plan = []
            self.plan_index = 0
            self.detected_objects = ["red cube", "blue sphere", "green bottle"] # Simulate object detection

            self.llm_lock = threading.Lock() # To prevent concurrent LLM calls

            # Timer to process next action
            self.action_timer = self.create_timer(1.0, self.execute_next_action_callback)
            
            # Simulate a voice command input
            self.simulated_voice_command_input("Robot, move forward 1 meter then report your status.")
            # self.simulated_voice_command_input("Robot, navigate to the red cube, pick it up, and place it down in the corner.")


        def simulated_voice_command_input(self, text_command: str):
            """
            Simulates a voice command by directly passing text to the LLM planning.
            In a real system, this would come from Whisper.
            """
            self.get_logger().info(f"Simulated voice command: '{text_command}'")
            self.plan_and_execute(text_command)

        def plan_and_execute(self, command_text: str):
            if not self.llm_lock.acquire(blocking=False):
                self.get_logger().warn("LLM already busy, skipping new command.")
                return

            try:
                self.get_logger().info("Requesting LLM plan...")
                self.current_plan = get_robot_plan_from_llm(command_text, self.detected_objects)
                self.plan_index = 0
                if self.current_plan:
                    self.get_logger().info(f"LLM generated plan: {self.current_plan}")
                else:
                    self.get_logger().error("LLM failed to generate a plan.")
            finally:
                self.llm_lock.release()

        def execute_next_action_callback(self):
            if not self.current_plan or self.plan_index >= len(self.current_plan):
                if self.current_plan and self.plan_index >= len(self.current_plan):
                    self.get_logger().info("Plan completed.")
                self.current_plan = [] # Reset plan
                return

            action = self.current_plan[self.plan_index]
            action_type = action.get("action")
            params = action.get("params", {})

            self.get_logger().info(f"Executing action: {action_type} with params: {params}")

            # Translate LLM action into ROS 2 command
            if action_type == "MOVE_FORWARD":
                twist_msg = Twist()
                twist_msg.linear.x = params.get("distance", 0.0) # For simplicity, treat distance as linear.x for a short duration
                self.cmd_vel_publisher.publish(twist_msg)
                self.get_logger().info(f"Published cmd_vel: linear.x={twist_msg.linear.x}")
                # In a real system, you'd track completion of this move
                
            elif action_type == "STOP":
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.cmd_vel_publisher.publish(twist_msg)
                self.get_logger().info("Published cmd_vel: STOP")

            elif action_type == "REPORT_STATUS":
                status_msg = String()
                status_msg.data = params.get("message", "Status reported.")
                self.status_publisher.publish(status_msg)
                self.get_logger().info(f"Published status: {status_msg.data}")

            elif action_type == "NAVIGATE_TO":
                # For this lab, we'll just print, as full Nav2 integration is complex
                self.get_logger().info(f"Simulating navigation to: {params.get('location')}")
                status_msg = String()
                status_msg.data = f"Navigating to {params.get('location')}"
                self.status_publisher.publish(status_msg)

            elif action_type == "PICK_UP":
                self.get_logger().info(f"Simulating picking up: {params.get('object')}")
                status_msg = String()
                status_msg.data = f"Attempting to pick up {params.get('object')}"
                self.status_publisher.publish(status_msg)

            elif action_type == "PLACE_DOWN":
                self.get_logger().info(f"Simulating placing down: {params.get('location')}")
                status_msg = String()
                status_msg.data = f"Attempting to place down at {params.get('location')}"
                self.status_publisher.publish(status_msg)

            elif action_type == "SCAN_FOR_OBJECTS":
                self.get_logger().info(f"Simulating scanning for objects. Currently detected: {self.detected_objects}")
                status_msg = String()
                status_msg.data = f"Scanning complete. Detected: {', '.join(self.detected_objects)}"
                self.status_publisher.publish(status_msg)

            elif action_type == "WAIT":
                wait_time = params.get("seconds", 1)
                self.get_logger().info(f"Simulating waiting for {wait_time} seconds. (Actual wait is non-blocking here)")
                # In a real system, this would block the action executor or use a timer

            else:
                self.get_logger().warn(f"Unknown action type: {action_type}")

            self.plan_index += 1

    def main(args=None):
        rclpy.init(args=args)
        node = VLASystemNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

## 4. Update `my_first_package/setup.py`

Add the new node to your `my_first_package/setup.py`'s `console_scripts` entry points:
```python
'console_scripts': [
    # ... other nodes ...
    'vla_system_node = my_first_package.vla_system:main',
],
```

## 5. Build and Run

1.  **Build your ROS 2 workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_first_package
    . install/setup.bash
    ```

2.  **Ensure you have an active ROS 2 environment**:
    -   You might want to run `ros2 run turtlesim turtlesim_node` in a separate terminal to visualize `cmd_vel` if you desire.

3.  **Run the VLA System Node**:
    ```bash
    ros2 run my_first_package vla_system_node
    ```

## Expected Behavior

-   The `vla_system_node` will start and immediately process the hardcoded initial command.
-   It will call the LLM API to get a plan.
-   It will then start executing the plan, publishing `cmd_vel` messages (for `MOVE_FORWARD`/`STOP`) and `robot_status` messages.
-   If you have Turtlesim or your `my_mobile_robot` in Gazebo running and listening to `cmd_vel`, you might see a response (e.g., the turtle moving).
-   Observe the terminal output logging the LLM plan and the execution of each action.

**Congratulations! You have successfully built a basic VLA pipeline that translates voice commands (simulated) into LLM plans and ROS 2 actions!** This is the core of an intelligent robot that can understand and react.

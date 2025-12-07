# Lab 11: LLM-Based Task Planning

## Objective
In this lab, you will explore the power of Large Language Models (LLMs) for high-level robot task planning. You will interact with a pre-trained LLM via its API to translate natural language commands into a structured sequence of robot-executable actions.

## Prerequisite Skills
- Basic Python programming.
- Understanding of making HTTP requests in Python (e.g., using `requests` library).
- An API key for an LLM (e.g., OpenAI, Google Gemini, Anthropic Claude).

---

## Part 1: Setting Up Your LLM Environment

### 1. Obtain an API Key

You will need an API key for a commercially available Large Language Model.
- **OpenAI**: Visit [platform.openai.com](https://platform.openai.com) and create an account. Generate a new secret key.
- **Google Gemini**: Visit [ai.google.dev](https://ai.google.dev) and obtain an API key.
- **Anthropic Claude**: Visit [console.anthropic.com](https://console.anthropic.com) and generate an API key.

Store your API key securely, preferably in an environment variable, and **never hardcode it directly into your script.**

### 2. Install Required Python Libraries

```bash
pip install openai # Or google-generativeai, anthropic, etc.
```

## Part 2: Interacting with the LLM for Task Planning

We will write a Python script that prompts an LLM to break down a high-level command into a list of robot actions.

1. Create a new Python file (e.g., `llm_planner.py`) in your preferred development directory (e.g., `~/vla_labs`).

2. Add the following Python code. This example uses OpenAI's API, but you can adapt it for other LLMs.

    ```python
    import os
    import openai
    import json

    # --- Configuration ---
    # Set your OpenAI API key as an environment variable or replace os.getenv()
    # Example: export OPENAI_API_KEY='sk-YOUR_KEY'
    openai.api_key = os.getenv("OPENAI_API_KEY")

    # Choose a suitable model
    # For OpenAI: "gpt-3.5-turbo", "gpt-4", "gpt-4o"
    # For others, check their API documentation.
    LLM_MODEL = "gpt-3.5-turbo" 

    # --- LLM Interaction Function ---
    def get_robot_plan_from_llm(user_command: str) -> list[str]:
        """
        Queries an LLM to generate a sequence of robot actions for a given command.
        """
        prompt = f"""
        You are a robot task planner. Your goal is to break down high-level human commands
        into a sequence of simple, robot-executable actions.

        The robot has the following capabilities (actions):
        - NAVIGATE_TO(location: str)
        - PICK_UP(object: str)
        - PLACE_DOWN(location: str)
        - REPORT_STATUS(message: str)
        - WAIT(seconds: int)
        - SCAN_FOR_OBJECTS()

        Please respond ONLY with a JSON array of these actions. Each action should be a JSON object
        with an "action" key and relevant "params" keys.

        Example:
        Human command: "Go to the kitchen and grab the coffee cup, then bring it to me in the living room."
        Robot plan:
        [
            {{"action": "NAVIGATE_TO", "params": {{"location": "kitchen"}}}},
            {{"action": "SCAN_FOR_OBJECTS", "params": {{}}}},
            {{"action": "PICK_UP", "params": {{"object": "coffee cup"}}}},
            {{"action": "NAVIGATE_TO", "params": {{"location": "living room"}}}},
            {{"action": "PLACE_DOWN", "params": {{"location": "table in living room"}}}},
            {{"action": "REPORT_STATUS", "params": {{"message": "Coffee cup delivered."}}}}
        ]

        Now, for the following human command: "{user_command}"
        Robot plan:
        """

        messages = [
            {"role": "system", "content": "You are a helpful assistant."}, 
            {"role": "user", "content": prompt}
        ]

        try:
            response = openai.ChatCompletion.create(
                model=LLM_MODEL,
                messages=messages,
                temperature=0.0 # Keep responses deterministic for planning
            )
            llm_response_content = response.choices[0].message['content'].strip()
            # Attempt to parse the JSON response
            return json.loads(llm_response_content)
        except json.JSONDecodeError as e:
            print(f"Error parsing LLM response JSON: {e}")
            print(f"LLM Response: {llm_response_content}")
            return []
        except Exception as e:
            print(f"An error occurred during LLM API call: {e}")
            return []

    # --- Main execution ---
    if __name__ == "__main__":
        print("Robot Task Planner (LLM-powered)")
        print("Type your command, or 'quit' to exit.")

        while True:
            human_command = input("\nHuman command: ")
            if human_command.lower() == 'quit':
                break

            print("\nGenerating robot plan...")
            robot_plan = get_robot_plan_from_llm(human_command)

            if robot_plan:
                print("\n--- Robot Plan ---")
                for i, action_step in enumerate(robot_plan):
                    action_type = action_step.get("action", "UNKNOWN")
                    params = action_step.get("params", {})
                    param_str = ", ".join([f"{k}: {v}" for k, v in params.items()])
                    print(f"{i+1}. {action_type}({param_str})")
            else:
                print("Could not generate a valid plan.")

    ```

## Part 3: Run and Test

1.  **Set your API Key**:
    -   Before running the script, make sure your LLM API key is set as an environment variable (`OPENAI_API_KEY` for OpenAI, or `GOOGLE_API_KEY`, etc. depending on your chosen LLM).
    -   Example for Linux/macOS: `export OPENAI_API_KEY='sk-xxxxxxxxxxxxxxxxxxxx'`
    -   Example for Windows PowerShell: `$env:OPENAI_API_KEY='sk-xxxxxxxxxxxxxxxxxxxx'`

2.  **Run the script**:
    ```bash
    python llm_planner.py
    ```

3.  **Enter commands**:
    -   When prompted, type a natural language command, e.g.:
        -   `"Find the red apple on the kitchen counter and bring it to the office desk."`
        -   `"Go check the status of the robot in the lab and report if it's working."`
        -   `"Just wait here for 5 seconds."`

## Expected Behavior

-   The script will send your natural language command to the LLM API.
-   The LLM will interpret the command and return a JSON array of robot actions based on the capabilities provided in the prompt.
-   The script will parse this JSON and print the sequence of actions in a human-readable format.
-   You should observe that the LLM is capable of breaking down complex instructions into a series of fundamental robot actions.

**Congratulations! You have successfully used an LLM to generate high-level task plans for a robot, demonstrating a core component of a VLA system!** In the next chapter, we will integrate this planning capability with actual perception and ROS 2 control.

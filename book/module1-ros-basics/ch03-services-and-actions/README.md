# Chapter 3: ROS 2 Services and Actions

## Introduction

In Chapter 2, you mastered the publisher-subscriber pattern using topics, which is perfect for continuous data streams. But what if you need a two-way conversation? What if you need to request a specific task to be done and wait for a confirmation that it's complete? Or what if you have a long-running goal, like "navigate to the kitchen," where you need to get periodic feedback and have the ability to cancel it?

For these scenarios, ROS 2 provides two other powerful communication patterns: **Services** and **Actions**. This chapter will teach you when and how to use them to build more robust and capable robotics applications.

## Learning Outcomes

By the end of this chapter, you will be able to:

-   Explain the difference between topics, services, and actions.
-   Identify use cases where services or actions are more appropriate than topics.
-   Create a custom service definition file (`.srv`).
-   Write a service server node that provides a service and a client node that calls it.
-   Create a custom action definition file (`.action`).
-   Write an action server node that executes a long-running task and an action client node that requests it.

## Key Concepts

### Services: Request/Response

A **Service** is a two-way, request-response communication pattern. It consists of two parts: a **request** and a **response**.

-   A **Service Server** node advertises a service by name. It waits for a request, performs a task, and then sends back a response.
-   A **Service Client** node calls the service by name, sends a request, and waits until it receives the response from the server.

This is a **synchronous** operation (from the client's perspective). The client code will block (wait) until the server's response is received. This is ideal for tasks that are quick to execute and require a confirmation, like "get the robot's current position" or "turn on the vacuum cleaner."

### Actions: Long-Running Tasks with Feedback

An **Action** is a more complex communication pattern designed for long-running, goal-oriented tasks that can be preempted. It provides a structured protocol for a client and server to communicate about a goal.

An action consists of three parts:

1.  **Goal**: The client sends a goal to the action server (e.g., "navigate to pose X," "rotate the arm by 90 degrees"). The server acknowledges the goal and starts executing it.
2.  **Feedback**: While the server is executing the goal, it can send a stream of updates, or feedback, back to the client (e.g., "current distance to goal is 5 meters," "percent complete: 75%").
3.  **Result**: When the task is finished, the server sends a final result to the client (e.g., "Goal reached successfully," "Failed to reach goal").

Unlike services, actions are **asynchronous**. The client doesn't have to wait for the goal to be completed. It can monitor the feedback or even cancel the goal mid-execution. This makes actions perfect for tasks like navigation, manipulation, or any process that takes more than a second or two.

## Important Definitions

-   **Service**: A synchronous, request-response communication pattern in ROS 2.
-   **Service Definition (`.srv`)**: A file that defines the data structure for a service's request and response parts.
-   **Service Server**: A node that offers a service to other nodes.
-   **Service Client**: A node that calls a service.
-   **Action**: An asynchronous communication pattern for long-running, preemptible goals with feedback.
-   **Action Definition (`.action`)**: A file that defines the data structure for an action's goal, result, and feedback parts.
-   **Action Server**: A node that executes goals for other nodes.
-   **Action Client**: A node that sends goals to an action server.

## Real-life Robotics Examples

-   **Service Example**:
    -   A `turtlesim` simulation node offers a `/reset` service. A client can call this service to instantly reset the simulation to its starting state. The server performs the reset and sends back an empty response to confirm it's done.
    -   A robot's hardware driver might offer a `/get_temperature` service that returns the current CPU temperature.

-   **Action Example**:
    -   The ROS 2 Navigation Stack (Nav2) provides an action server called `/navigate_to_pose`. A client sends a goal with the desired (x, y, theta) coordinates. The server then starts moving the robot, periodically publishing feedback with the robot's current distance to the goal. When the robot arrives, the server sends a final result. The client can cancel the navigation goal at any time.
    -   A robotic arm might have a `/follow_joint_trajectory` action to execute a complex multi-point movement.

## Required Python/ROS 2 Skills

-   Experience creating ROS 2 packages and nodes (from Chapter 2).
-   Ability to build and run packages with `colcon`.

## Hands-on Lab Summary

This chapter's lab will guide you through creating both a service and an action.

1.  **Service**: You will create a service that takes two integers and returns their sum. You'll write a server node to perform the addition and a client node to request the calculation. This simple example will solidify your understanding of the request-response pattern.
2.  **Action**: You will implement an action that simulates a countdown. The action client will send a goal (the number of seconds to count down from). The action server will "count down" for the requested duration, sending feedback every second. When finished, it will send a result message. This will teach you how to handle the asynchronous nature of actions and their feedback mechanism.

*(The detailed, step-by-step lab instructions will be in a separate file in the `labs/` directory.)*

## Summary

You have now learned about all three primary communication methods in ROS 2: Topics, Services, and Actions. You know when to use each one: Topics for continuous data streams, Services for quick, synchronous request-response tasks, and Actions for long-running, asynchronous goals. With this knowledge, you are equipped to design and build much more complex and robust robotic systems. In the next chapter, we will shift our focus from communication to representation, learning how to model a physical robot in software using URDF.

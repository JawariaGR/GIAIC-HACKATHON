# Chapter 2: ROS 2 Nodes, Topics, and Messages

## Introduction

In the last chapter, you successfully set up your ROS 2 environment and ran a simple simulation. You interacted with ROS using command-line tools, but you didn't write any code. Now, it's time to change that. This chapter dives into the most fundamental concept in ROS: the communication graph. We will explore how individual ROS programs (Nodes) communicate with each other by passing data (Messages) over named channels (Topics). Understanding this publisher-subscriber pattern is the key to building any robotics application in ROS.

## Learning Outcomes

By the end of this chapter, you will be able to:

-   Explain the roles of nodes, topics, and messages in the ROS 2 graph.
-   Create a new ROS 2 package to hold your own code.
-   Write a simple publisher node in Python that sends data.
-   Write a simple subscriber node in Python that receives and processes data.
-   Use command-line tools like `ros2 node`, `ros2 topic`, and `ros2 interface` to inspect the communication graph.

## Key Concepts

### The ROS 2 Graph

The "graph" is the network of ROS 2 elements processing data together. It consists of:

1.  **Nodes**: A node is the smallest unit of executable code in ROS. Think of it as a single program or process with a specific purpose (e.g., a node to control a camera, a node to plan a path, a node to control wheel motors). Each node in a ROS system should be responsible for a single, module purpose.
2.  **Topics**: Topics are the "pipes" or named buses over which nodes exchange data. They are the streets on a city map. A node can "publish" data to a topic, and any number of other nodes can "subscribe" to that same topic to receive the data. This decouples data production from data consumption, which is a powerful design pattern.
3.  **Messages**: When a node publishes data to a topic, it sends the data in the form of a message. A message is simply a data structure, like a C `struct` or a Python `class`, with a defined type and fields. For example, a `Twist` message from the `geometry_msgs` package has fields for linear and angular velocity, which is perfect for sending drive commands to a robot.

### Publisher/Subscriber Pattern

This is the primary communication method in ROS.

-   A **Publisher** is a node that sends out data on a topic. It defines the topic name and the message type.
-   A **Subscriber** is a node that receives data on a topic. It "listens" for any messages published on a specific topic.

Crucially, the publisher and subscriber nodes don't know about each other. The publisher sends data to the topic, and the ROS 2 middleware (DDS) ensures that the data is delivered to all subscribers of that topic. This makes the system very modular and scalable. You can add or remove subscribers without affecting the publisher at all.

## Important Definitions

-   **Node**: An executable program that connects to the ROS 2 graph to communicate with other nodes.
-   **Topic**: A named channel for communication. Nodes publish messages to topics, and other nodes subscribe to topics to receive those messages.
-   **Message**: A data structure with a specific type that defines the data being sent over a topic.
-   **Package**: The primary unit for organizing software in ROS. A package can contain ROS nodes, libraries, configuration files, and more.
-   **rclpy**: The ROS 2 client library for Python. It provides the essential functions and classes to write ROS 2 nodes in Python.
-   **colcon**: The standard build tool for ROS 2. It is used to compile and install ROS 2 packages in your workspace.

## Real-life Robotics Examples

-   **Camera Driver**: A camera driver node could publish raw image data (e.g., a `sensor_msgs/Image` message) to an `/image_raw` topic.
-   **Object Detection**: An object detection node could subscribe to the `/image_raw` topic, process the images to find objects, and then publish the locations of those objects (e.g., a custom `BoundingBox` message) to an `/object_detections` topic.
-   **Motor Controller**: A motor controller node could subscribe to a `/cmd_vel` topic (which uses a `geometry_msgs/Twist` message) and translate the velocity commands into actual motor signals.

## Required Python/ROS 2 Skills

-   Basic Python programming (variables, functions, classes).
-   Understanding of the `ros2 run` and `ros2 topic` commands.

## Hands-on Lab Summary

In this chapter's lab, you will put theory into practice by building your very first ROS 2 package from scratch. Inside this package, you will write two Python nodes:
1.  A "talker" node that periodically publishes a `String` message with a "hello world" counter to a topic.
2.  A "listener" node that subscribes to the same topic and prints the received message to the console.

You will learn how to use `colcon` to build your package and `ros2 run` to execute your new nodes. This simple "talker-listener" example is a canonical ROS exercise that demonstrates the core publisher-subscriber communication pattern.

*(The detailed, step-by-step lab instructions will be in a separate file in the `labs/` directory.)*

## Summary

This chapter covered the absolute fundamentals of ROS 2 communication. You learned about the graph architecture and the roles of nodes, topics, and messages. You now understand the powerful publisher-subscriber pattern that enables modularity in robotics software. Most importantly, you are about to write and run your own ROS 2 nodes. In the next chapter, we will explore two other communication patterns—Services and Actions—which are used for request-response and long-running tasks.

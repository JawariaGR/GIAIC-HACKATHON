# Chapter 4: URDF and Robot Modeling

## Introduction

So far, we have learned how different parts of a ROS 2 system can communicate with each other. But we haven't yet talked about the robot itself! How do we describe a physical robot so that software can understand it? How do we tell a program where the robot's wheels are, how its arm is jointed, or what it looks like?

The answer is the **Unified Robot Description Format (URDF)**. URDF is an XML-based file format used in ROS to describe all the physical elements of a robot. This chapter will teach you how to read, understand, and write URDF files to create a "digital twin" of your robot's structure.

## Learning Outcomes

By the end of this chapter, you will be able to:

-   Explain the purpose of a URDF file.
-   Describe the core components of a URDF: `<link>` and `<joint>`.
-   Define the visual and collision properties of a robot's parts.
-   Understand the different joint types (e.g., `revolute`, `continuous`, `prismatic`, `fixed`).
-   Create a simple URDF file from scratch for a multi-link robot.
-   Use **RViz2** and `robot_state_publisher` to visualize and interact with your URDF model.

## Key Concepts

### `<link>`: The Physical Parts

A **link** represents a rigid part of the robot's body. It has physical properties like mass, inertia, and visual and collision geometry. Think of the links as the "bones" of the robot. For a simple mobile robot, the main chassis could be one link, and each wheel could be a separate link.

A link is defined by its:
-   **`<visual>`**: What the link looks like. This is used for visualization in tools like RViz and Gazebo. You can use simple shapes (box, cylinder, sphere) or complex 3D meshes (like `.dae` or `.stl` files).
-   **`<collision>`**: What the link's collision geometry is. This is used by physics engines to calculate collisions. It's often a simpler shape than the visual geometry to save computation.
-   **`<inertial>`**: The link's mass and rotational inertia. This is crucial for accurate physics simulation.

### `<joint>`: The Connections Between Parts

A **joint** describes the kinematic and dynamic relationship between two links. It defines how one link (the `child`) moves relative to another (the `parent`). Joints are the "muscles" or "articulations" of the robot.

A joint is defined by its:
-   **`type`**: The most important property. It defines the joint's motion.
    -   `revolute`: A hinge joint that rotates around a single axis with defined limits (e.g., an elbow).
    -   `continuous`: A joint that rotates continuously without limits (e.g., a wheel).
    -   `prismatic`: A sliding joint that moves along a single axis with limits (e.g., a piston).
    -   `fixed`: A joint that doesn't allow any movement. This is used to rigidly connect two links together.
-   **`<parent>` and `<child>`**: The two links that the joint connects.
-   **`<origin>`**: The transform (position and orientation) of the child link's frame relative to the parent link's frame.
-   **`<axis>`**: The axis of rotation or translation for `revolute`, `continuous`, and `prismatic` joints.

### Robot State and TF2

A URDF file only describes the *static* model of the robot. It doesn't know the current angle of its joints or its position in the world.

To bring the model to life, we use two key ROS 2 packages:
1.  **`robot_state_publisher`**: This node reads the URDF file and subscribes to a topic (usually `/joint_states`) that publishes the current state (e.g., angle, position) of each joint. It then calculates the 3D pose of every link in the robot and publishes these poses as **transforms**.
2.  **TF2 (Transform Library)**: TF2 is the ROS 2 library that manages transforms. It lets you ask questions like, "What is the position of the robot's hand relative to its base?" The `robot_state_publisher` uses TF2 to broadcast all the link poses.

### RViz2: The Robot Visualizer

**RViz2** is a powerful 3D visualization tool for ROS 2. It can subscribe to topics and use TF2 to display all sorts of data. We will use it to display our URDF model, showing a 3D representation of the robot based on the transforms published by `robot_state_publisher`.

## Important Definitions

-   **URDF (Unified Robot Description Format)**: An XML format for describing the physical model of a robot.
-   **Link**: A rigid part of a robot's body with physical properties.
-   **Joint**: Connects two links and defines their relative motion.
-   **Transform (TF)**: A representation of the position and orientation of one coordinate frame relative to another.
-   **RViz2**: A 3D visualization tool for ROS 2.
-   **`robot_state_publisher`**: A ROS 2 node that uses a URDF and joint states to publish the 3D poses of a robot's links.

## Hands-on Lab Summary

In this chapter's lab, you will get hands-on experience creating a URDF file from scratch. You will model a simple 2-DOF (Degrees of Freedom) robotic arm. You will define two arm segments (`link`s) and two rotating joints (`revolute` `joint`s).

Once the URDF is written, you will create a ROS 2 launch file to start the `robot_state_publisher` and RViz2. You will also use a simple GUI tool to publish fake `/joint_states` messages, allowing you to move the joints of your virtual robot arm and see it update live in RViz2. This provides instant, visual feedback on the URDF you've created.

*(The detailed, step-by-step lab instructions will be in a separate file in the `labs/` directory.)*

## Summary

This chapter introduced you to the crucial process of modeling a robot's physical structure for software. You learned about the core URDF components, links and joints, and how `robot_state_publisher` and RViz2 work together to bring your model to life. Being able to define a robot in a format that ROS understands is a prerequisite for almost all advanced topics, including simulation, navigation, and manipulation. In the next chapter, we will take the URDF model you've built and bring it into a full physics simulator, Gazebo.

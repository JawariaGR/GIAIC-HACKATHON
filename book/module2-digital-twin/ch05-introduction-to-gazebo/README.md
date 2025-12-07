# Chapter 5: Introduction to Gazebo

## Introduction

In the previous chapter, you learned how to describe a robot's physical structure using URDF and visualize it in RViz. While RViz is excellent for visualizing sensor data and robot poses, it has a major limitation: it's not a simulator. It doesn't understand physics, gravity, or collisions. To test how a robot will *behave* in the real world, we need a true robotics simulator.

Enter **Gazebo**. Gazebo is a powerful, open-source 3D robotics simulator. It allows you to place your URDF models in a simulated environment and watch how they interact with physics. You can add lights, gravity, and other objects. Crucially, Gazebo is tightly integrated with ROS 2, allowing your ROS nodes to seamlessly control simulated robots just as they would control real ones. This chapter is your first step into the world of high-fidelity simulation.

## Learning Outcomes

By the end of this chapter, you will be able to:

-   Explain the difference between a visualizer (RViz) and a simulator (Gazebo).
-   Understand the architecture of Gazebo and how it integrates with ROS 2.
-   Launch pre-existing Gazebo worlds.
-   Spawn your URDF robot model from Chapter 4 into a Gazebo simulation.
-   Use ROS 2 command-line tools to interact with and control your simulated robot.

## Key Concepts

### Why Simulation?

Developing on physical robots is slow, expensive, and risky. If you have a bug in your control code, you could damage a multi-thousand-dollar robot. Simulation provides a safe, fast, and cost-effective alternative.

-   **Safety**: Test dangerous maneuvers without risk to hardware or people.
-   **Speed**: Run tests faster than real-time.
-   **Parallelization**: Run thousands of simulations in the cloud simultaneously to test different scenarios.
-   **Accessibility**: Anyone can run a simulation, even without access to an expensive physical robot.

### Gazebo Architecture

Gazebo runs as two main processes:
1.  **Gazebo Server (`gzserver`)**: This is the core of the simulator. It runs the physics engine, simulates the sensors, and renders the world. It can be run "headless" (without a graphical interface) on a server.
2.  **Gazebo Client (`gzclient`)**: This is the graphical user interface (GUI). It connects to the `gzserver` and provides a 3D visualization of the simulated world. You can use it to inspect models, interact with the environment, and visualize sensor data.

### Gazebo and ROS 2 Integration

The `gazebo_ros_pkgs` package provides the bridge between Gazebo and ROS 2. It allows Gazebo to function like a collection of ROS 2 nodes.

-   **Simulation Time**: Gazebo publishes the simulation time to the `/clock` topic. By setting the `use_sim_time` parameter to `true` in all your ROS 2 nodes, you ensure that your entire system runs on Gazebo's clock, not the real-world clock. This allows the simulation to be paused, slowed down, or sped up while the ROS system remains perfectly synchronized.
-   **Plugins**: Gazebo uses plugins to expose robot models and sensors as ROS 2 topics, services, and actions. For example, a "differential drive" plugin can subscribe to a `/cmd_vel` topic and convert the velocity commands into forces that move the wheels of your simulated robot. A "camera" plugin can generate simulated camera images and publish them to a ROS 2 topic.

### Worlds and Models

-   **World (`.world`)**: A Gazebo world is an XML-like file in the **SDF (Simulation Description Format)** that describes the entire environment. This includes the lighting, physics properties (like gravity), and any static objects like buildings, tables, or obstacles.
-   **Model (`.sdf`/`.urdf`)**: A model represents a single object in the simulation, such as a robot, a table, or a coke can. While Gazebo's native format is SDF, it has excellent support for importing ROS URDF files.

## Important Definitions

-   **Simulator**: A software tool that models a real-world system, including its physics and interactions.
-   **Gazebo**: A popular, open-source 3D robotics simulator.
-   **SDF (Simulation Description Format)**: The XML-based format used by Gazebo to describe worlds, models, and robots.
-   **Plugin**: A piece of code that can be added to Gazebo to extend its functionality, such as by providing a ROS 2 interface for a sensor or actuator.
-   **`use_sim_time`**: A ROS 2 parameter that tells a node to use the time published on the `/clock` topic instead of the system's wall clock. This is essential for working with simulators.

## Hands-on Lab Summary

In this lab, you will take the 2-DOF robotic arm you created in Chapter 4 and bring it into a full physics simulation. You will:
1.  Create a new launch file to start both Gazebo and RViz.
2.  Use the `spawn_entity.py` node to load your URDF file and add it to the Gazebo simulation.
3.  Verify that you can see the arm in both the Gazebo window (with physics) and the RViz window (for ROS visualization).
4.  Use the `joint_state_publisher_gui` to send joint commands and observe how your arm now realistically swings and is affected by gravity in Gazebo.

*(The detailed, step-by-step lab instructions will be in a separate file in the `labs/` directory.)*

## Summary

This chapter was your first foray into high-fidelity physics simulation. You learned why simulation is a critical part of modern robotics development and were introduced to the architecture of Gazebo. You now understand how Gazebo integrates with ROS 2 to create a powerful "hardware-in-the-loop"-style development environment where your ROS code doesn't need to know whether it's talking to a real robot or a simulated one. In the next chapter, we will expand on this by adding simulated sensors and actuators to our robot, allowing it to perceive and interact with its virtual world.

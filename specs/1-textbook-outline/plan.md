# Implementation Plan: Detailed Textbook Outline

**Branch**: `1-textbook-outline` | **Date**: 2025-12-05 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `specs/1-textbook-outline/spec.md`

## Summary

This plan outlines the creation of a detailed, chapter-by-chapter structure for the "Physical AI & Humanoid Robotics" textbook. The goal is to produce a comprehensive outline that adheres to the feature specification, covering a 13-week learning path across four modules: ROS 2 Basics, Digital Twins, NVIDIA Isaac, and VLA Systems. Each chapter will be designed to be beginner-friendly, hands-on, and focused on practical robotics engineering.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**: ROS 2 Humble (LTS), Gazebo, Unity, NVIDIA Isaac Sim, `rclpy`, `colcon`
**Documentation Engine**: Docusaurus
**Storage**: Content will be managed by Docusaurus (Markdown, MDX), alongside Python files and simulation assets.
**Testing**: `colcon test` for ROS 2 packages, `pytest` for standalone Python examples
**Target Platform**: Ubuntu 22.04 (recommended for ROS 2 Humble compatibility)
**Project Type**: Educational Content (Textbook with code, labs, and simulations)
**Performance Goals**: Simulations must run effectively on mid-range hardware (e.g., modern CPU, 16GB RAM, NVIDIA RTX 30-series GPU).
**Constraints**: The material must be accessible to students with basic Python knowledge but little to no robotics experience.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **I. Beginner-Friendly & Python-Based**: The plan centers on Python and breaking down complex topics for students.
- [x] **II. Engineering-Focused & Hands-On**: Every chapter includes a hands-on lab and simulation exercise.
- [x] **III. Modular & Structured Learning Path**: The plan is built around the specified 4-module, 13-week structure.
- [x] **IV. End-to-End System Focus**: The plan culminates in a capstone project integrating all learned concepts.
- [x] **V. Simulation-First with Digital Twins**: Gazebo, Unity, and Isaac Sim are central to the development and learning workflow.

## Detailed Chapter-by-Chapter Plan

Here is the proposed 13-week outline.

---

### **Module 1: ROS 2 and Robotics Fundamentals (Weeks 1-4)**

**Goal**: Equip students with the foundational knowledge of ROS 2 and core robotics concepts.

*   **Chapter 1: Introduction to Robotics and ROS 2**
    *   **Learning Outcomes**: Understand the components of a robotic system; explain what ROS is and why it's used; set up a ROS 2 development environment.
    *   **Lab**: Install ROS 2 Humble on Ubuntu 22.04; run a "turtlesim" simulation and control the turtle with command-line tools.

*   **Chapter 2: ROS 2 Nodes, Topics, and Messages**
    *   **Learning Outcomes**: Create a ROS 2 package and a Python node; understand the publisher/subscriber pattern; use topics to pass data between nodes.
    *   **Lab**: Write a Python publisher node that sends velocity commands and a subscriber node that reads and prints them.

*   **Chapter 3: ROS 2 Services and Actions**
    *   **Learning Outcomes**: Differentiate between topics, services, and actions; implement a request/response pattern with services; manage long-running tasks with actions.
    *   **Lab**: Create a service that resets a simulation and an action that makes a robot move to a specific goal.

*   **Chapter 4: URDF and Robot Modeling**
    *   **Learning Outcomes**: Understand what URDF is and how it's used; define robot links, joints, and visuals; view a robot model in RViz2.
    *   **Lab**: Build a URDF model for a simple robotic arm (e.g., 2-DOF) and visualize it.

---

### **Module 2: Simulation and Digital Twins (Weeks 5-7)**

**Goal**: Teach students to create and use high-fidelity simulations for robot development.

*   **Chapter 5: Introduction to Gazebo**
    *   **Learning Outcomes**: Understand the role of a physics simulator; spawn a URDF model in a Gazebo world; interact with the simulated robot.
    *   **Lab**: Spawn the robotic arm from Chapter 4 in Gazebo; add a simple world with ground and objects.

*   **Chapter 6: Sensors and Actuators in Gazebo**
    *   **Learning Outcomes**: Add sensor plugins (LiDAR, depth camera, IMU) to a Gazebo model; control joints using Gazebo plugins and ROS 2 topics.
    *   **Lab**: Add a LiDAR sensor to a simple mobile robot model; visualize the sensor data in RViz2; write a node to drive the robot.

*   **Chapter 7: Advanced Simulation with Unity**
    *   **Learning Outcomes**: Understand the benefits of a high-fidelity rendering engine like Unity for robotics; set up the ROS-Unity integration.
    *   **Lab**: Import the robotic arm model into a Unity scene; control its joints using ROS 2 messages.

---

### **Module 3: NVIDIA Isaac for Perception and AI (Weeks 8-10)**

**Goal**: Introduce advanced perception and AI tools using the NVIDIA Isaac platform.

*   **Chapter 8: Introduction to NVIDIA Isaac Sim**
    *   **Learning Outcomes**: Understand the capabilities of Isaac Sim; set up the Isaac Sim environment; load and control a robot.
    *   **Lab**: Load a pre-built Franka Emika robot in Isaac Sim and control it via its ROS 2 interface.

*   **Chapter 9: Perception and VSLAM**
    *   **Learning Outcomes**: Understand the principles of computer vision and Simultaneous Localization and Mapping (SLAM); use Isaac Sim's perception tools.
    *   **Lab**: Implement a basic VSLAM pipeline using data from a simulated robot's camera in Isaac Sim to build a map of the environment.

*   **Chapter 10: Navigation with Nav2**
    *   **Learning Outcomes**: Understand the components of the Nav2 stack; configure Nav2 for a custom robot; perform autonomous point-to-point navigation.
    *   **Lab**: Configure and launch Nav2 for a mobile robot in Isaac Sim; give it a goal pose and watch it navigate autonomously.

---

### **Module 4: Vision-Language-Action Systems (Weeks 11-13)**

**Goal**: Integrate large-scale AI models to create intelligent, responsive robots.

*   **Chapter 11: Introduction to VLA Systems**
    *   **Learning Outcomes**: Understand the architecture of a Vision-Language-Action system; learn how LLMs can be used for task planning.
    *   **Lab**: Use a pre-trained LLM (via an API) to convert a natural language command (e.g., "pick up the red cube") into a sequence of robot actions.

*   **Chapter 12: Building the VLA Pipeline**
    *   **Learning Outcomes**: Integrate Whisper for speech-to-text, an LLM for planning, and object detection for perception; connect the pipeline to ROS 2.
    *   **Lab**: Build a Python application that takes a voice command, generates a plan, and publishes the first action step to a ROS 2 topic.

*   **Chapter 13: Capstone Project**
    *   **Learning Outcomes**: Integrate all concepts from the book to build a complete end-to-end system.
    *   **Lab**: Assemble the full capstone project: a humanoid robot in Isaac Sim that listens to a voice command, plans a task, navigates to an object, detects it, manipulates it, and provides a spoken response.

## Project Structure

### Source Code (repository root)

The project will be organized as a Docusaurus website with supplementary simulation and code assets.

```text
website/
├── docusaurus.config.js  # Docusaurus site configuration
├── package.json
├── sidebars.js           # Defines the documentation sidebar structure
├── src/
│   ├── css/
│   └── pages/
└── docs/                   # All textbook content resides here
    ├── module1-ros-basics/
    │   ├── ch01-introduction/
    │   │   ├── _category_.json # Defines sidebar category properties
    │   │   ├── index.md        # Chapter text
    │   │   └── labs.md         # Page for lab instructions
    │   └── ...
    ├── module2-digital-twin/
    ├── module3-nvidia-isaac/
    └── module4-vla-systems/
simulations/                  # Simulation assets (worlds, models)
├── gazebo_worlds/
└── urdf/
code_labs/                    # All hands-on lab code
├── module1/
│   └── ch01/
│       └── ...
└── ...
```

**Structure Decision**: Using Docusaurus provides a robust, searchable, and professional-looking website for the textbook. Separating the website (`website/`), simulation assets (`simulations/`), and lab code (`code_labs/`) creates a clean and maintainable project structure.

## Complexity Tracking

No constitutional violations detected. The plan directly implements the principles.

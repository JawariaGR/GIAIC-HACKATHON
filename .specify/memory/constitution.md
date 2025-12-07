<!--
---
sync_impact_report:
  version_change: "0.0.0 -> 1.0.0"
  principles_modified:
    - "Principle 1: Beginner-Friendly & Python-Based (new)"
    - "Principle 2: Engineering-Focused & Hands-On (new)"
    - "Principle 3: Modular & Structured Learning Path (new)"
    - "Principle 4: End-to-End System Focus (new)"
    - "Principle 5: Simulation-First with Digital Twins (new)"
  sections_added:
    - "Core Principles"
    - "Development Workflow"
    - "Governance"
  sections_removed: []
  templates_updated:
    - path: ".specify/memory/constitution.md"
      status: "updated"
    - path: ".specify/templates/plan-template.md"
      status: "pending"
    - path: ".specify/templates/spec-template.md"
      status: "pending"
    - path: ".specify/templates/tasks-template.md"
      status: "pending"
  todos: []
---
-->
# Physical AI & Humanoid Robotics Constitution

This document outlines the core principles and standards for the "Physical AI & Humanoid Robotics" textbook project. Its purpose is to ensure the final product is cohesive, high-quality, and meets the educational goals for its intended audience.

## Core Principles

### I. Beginner-Friendly & Python-Based
All content, from conceptual explanations to code examples, MUST be created with a student audience in mind. The primary programming language is Python. Code MUST be well-commented, example-driven, and easy for a beginner to understand and execute. Complex topics should be broken down into digestible parts.

### II. Engineering-Focused & Hands-On
Every chapter MUST provide practical, applicable knowledge. This is achieved by including conceptual theory, real-world robotics scenarios, hands-on labs with ROS 2 and Python, detailed simulation steps, chapter quizzes, and summaries. The focus is on building, not just theorizing.

### III. Modular & Structured Learning Path
The book's content MUST adhere to the defined 13-week, 4-module structure. Each module (ROS 2 Basics, Digital Twin, NVIDIA Isaac, VLA Systems) must be self-contained enough to be understood on its own, but also build logically on the previous one.

### IV. End-to-End System Focus
The ultimate goal is to teach the construction of a complete, integrated system. The project MUST guide the student toward the capstone: a humanoid robot that integrates voice commands, LLM-based planning, ROS 2 control, navigation, perception, and manipulation into a single, functional demonstration.

### V. Simulation-First with Digital Twins
Development and validation of robotic systems MUST prioritize high-fidelity simulation. Tools like Gazebo, Unity, and NVIDIA Isaac are to be used to create digital twins of robots and environments. This allows for robust testing of perception, planning, and control algorithms before any deployment on physical hardware. Sim-to-real transfer is a key consideration.

## Development Workflow

- **Specification-Driven**: New chapters or significant changes start with a clear specification (`spec.md`) outlining learning objectives, content, and labs.
- **Planning**: An implementation plan (`plan.md`) details the technical approach, required assets (code, diagrams, simulations), and aligns with the constitution.
- **Task-Based Execution**: Work is broken down into small, verifiable tasks tracked in `tasks.md`.

## Governance

This constitution is the authoritative guide for the project. All contributions, reviews, and decisions must align with these principles.

- **Amendment Process**: Changes to this constitution require a proposal, review, and consensus from the core contributors. All amendments must be documented with a version increment.
- **Compliance**: All submitted content (text, code, simulations) will be reviewed for compliance with these principles.

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05
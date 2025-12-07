---
id: 100
title: Establish Project Constitution for Physical AI Textbook
stage: constitution
date: 2025-12-05
surface: agent
model: gemini-pro
feature: none
branch:
user:
command: /sp.constitution
labels: [constitution, project-setup]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
tests:

---

## Prompt

/sp.constitution
Physical AI & Humanoid Robotics is a full AI-driven textbook designed for students learning embodied intelligence. The book explains how digital AI models interact with the physical world through humanoid robots using ROS 2, Gazebo, Unity, and NVIDIA Isaac. It focuses on how robots perceive, plan, and act using sensors, actuators, AI models, and simulation environments.

The book is beginner-friendly, Python-based, example-driven, and engineering-focused. Each chapter includes conceptual explanation, real robotics scenarios, hands-on labs, ROS 2 + Python code, simulation steps, quizzes, and summaries.

Structure follows a 13-week learning path:
Module 1: ROS 2 basics, nodes, topics, services, actions, URDF, rclpy, launch files.
Module 2: Digital Twin (Gazebo + Unity), physics simulation, joints, collisions, LiDAR, depth camera, IMU.
Module 3: NVIDIA Isaac, perception, VSLAM, navigation (Nav2), reinforcement learning, sim-to-real.
Module 4: Vision-Language-Action (VLA) systems using Whisper, LLMs, object detection, path planning, grasping.

Capstone: A humanoid robot that listens to voice commands, uses Whisper -> LLM planning -> ROS 2 control -> navigation -> object detection -> manipulation -> response.

End goal: Teach students to build real humanoid intelligence by connecting digital AI models with physical robots.

## Response snapshot

I have successfully established the project constitution.

- Created the initial project constitution at `.specify/memory/constitution.md` (Version 1.0.0).
- Defined five core principles based on your project description:
    1.  Beginner-Friendly & Python-Based
    2.  Engineering-Focused & Hands-On
    3.  Modular & Structured Learning Path
    4.  End-to-End System Focus
    5.  Simulation-First with Digital Twins
- Updated the plan template at `.specify/templates/plan-template.md` to include a constitution checklist, making our principles actionable.
- Created a Prompt History Record (PHR) to document this action.

## Outcome

- ✅ Impact: Established the foundational principles for the project.
- 🧪 Tests: None
- 📁 Files: 
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
- 🔁 Next prompts: {{NEXT_PROMPTS}}
- 🧠 Reflection: {{REFLECTION_NOTE}}

## Evaluation notes (flywheel)

- Failure modes observed: {{FAILURE_MODES}}
- Graders run and results (PASS/FAIL): {{GRADER_RESULTS}}
- Prompt variant (if applicable): {{PROMPT_VARIANT_ID}}
- Next experiment (smallest change to try): {{NEXT_EXPERIMENT}}

# Tasks: Detailed Textbook Outline (Docusaurus Implementation)

**Input**: Design documents from `specs/1-textbook-outline/`
**Prerequisites**: `plan.md` (required)

**Organization**: Tasks are grouped by phase, starting with Docusaurus setup, followed by content creation for each module.

---

## Phase 1: Docusaurus Project Setup

**Purpose**: Initialize the Docusaurus website and create the foundational directory structure.

- [x] T001: Initialize a new Docusaurus site (classic theme) in a `website/` directory.
- [x] T002: Create the root `simulations/` and `simulations/urdf/` directories for simulation assets.
- [x] T003: Create the root `code_labs/` directory to store all hands-on lab code.
- [x] T004: Modify `docusaurus.config.js` with the project's title ("Physical AI & Humanoid Robotics"), tagline, and a basic navigation structure.
- [x] T005: Clean up the default Docusaurus `docs/` and `src/pages/` directories, removing placeholder content.

---

## Phase 2: Module 1 - ROS 2 and Robotics Fundamentals

**Purpose**: Develop the content and labs for the first four chapters, covering ROS 2 basics, within the Docusaurus structure.

- [x] T101: Create content file for Chapter 1 at `website/docs/module1-ros-basics/ch01-introduction/index.md`.
- [x] T102: Create lab page for Chapter 1 at `website/docs/module1-ros-basics/ch01-introduction/labs.md` and initial lab code under `code_labs/module1/ch01/`.
- [x] T103: Create content file for Chapter 2 at `website/docs/module1-ros-basics/ch02-nodes-and-topics/index.md`.
- [x] T104: Create lab page for Chapter 2 and initial lab code under `code_labs/module1/ch02/`.
- [x] T105: Create content file for Chapter 3 at `website/docs/module1-ros-basics/ch03-services-and-actions/index.md`.
- [x] T106: Create lab page for Chapter 3 and initial lab code under `code_labs/module1/ch03/`.
- [x] T107: Create content file for Chapter 4 at `website/docs/module1-ros-basics/ch04-urdf-and-robot-modeling/index.md`.
- [x] T108: Create lab page for Chapter 4 and initial lab code under `code_labs/module1/ch04/`.

---

## Phase 3: Module 2 - Simulation and Digital Twins

**Purpose**: Develop content and labs for chapters 5-7, focusing on Gazebo and Unity, within the Docusaurus structure.

- [x] T201: Create content file for Chapter 5 at `website/docs/module2-digital-twin/ch05-introduction-to-gazebo/index.md`.
- [x] T202: Create lab page for Chapter 5 and initial lab code under `code_labs/module2/ch05/`.
- [x] T203: Create content file for Chapter 6 at `website/docs/module2-digital-twin/ch06-sensors-and-actuators-in-gazebo/index.md`.
- [x] T204: Create lab page for Chapter 6 and initial lab code under `code_labs/module2/ch06/`.
- [x] T205: Create content file for Chapter 7 at `website/docs/module2-digital-twin/ch07-advanced-simulation-with-unity/index.md`.
- [x] T206: Create lab page for Chapter 7 and initial lab code under `code_labs/module2/ch07/`.

---

## Phase 4: Module 3 - NVIDIA Isaac for Perception and AI

**Purpose**: Develop content and labs for chapters 8-10, focusing on advanced simulation and perception, within the Docusaurus structure.

- [x] T301: Create content file for Chapter 8 at `website/docs/module3-nvidia-isaac/ch08-introduction-to-nvidia-isaac-sim/index.md`.
- [x] T302: Create lab page for Chapter 8 and initial lab code under `code_labs/module3/ch08/`.
- [x] T303: Create content file for Chapter 9 at `website/docs/module3-nvidia-isaac/ch09-perception-and-vslam/index.md`.
- [x] T304: Create lab page for Chapter 9 and initial lab code under `code_labs/module3/ch09/`.
- [x] T305: Create content file for Chapter 10 at `website/docs/module3-nvidia-isaac/ch10-navigation-with-nav2/index.md`.
- [x] T306: Create lab page for Chapter 10 and initial lab code under `code_labs/module3/ch10/`.

---

## Phase 5: Module 4 - Vision-Language-Action Systems

**Purpose**: Develop content and labs for chapters 11-13, focusing on integrating large AI models, within the Docusaurus structure.

- [x] T401: Create content file for Chapter 11 at `website/docs/module4-vla-systems/ch11-introduction-to-vla-systems/index.md`.
- [x] T402: Create lab page for Chapter 11 and initial lab code under `code_labs/module4/ch11/`.
- [x] T403: Create content file for Chapter 12 at `website/docs/module4-vla-systems/ch12-building-the-vla-pipeline/index.md`.
- [x] T404: Create lab page for Chapter 12 and initial lab code under `code_labs/module4/ch12/`.
- [x] T405: Create content file for Chapter 13 at `website/docs/module4-vla-systems/ch13-capstone-project/index.md`.
- [x] T406: Create lab page for Chapter 13 and initial lab code under `code_labs/module4/ch13/`.

---

## Phase 6: Finalization and Review

**Purpose**: Polish the website, review all code and labs, and ensure consistency.

- [x] T501: [P] Review and edit all chapter content for clarity, grammar, and technical accuracy.
- [x] T502: [P] Test all lab exercises from start to finish to ensure they are working correctly.
- [x] T503: [P] Validate all code snippets and simulation models.
- [x] T504: Build and serve the Docusaurus site locally to verify the final output.
- [x] T505: Add a concluding chapter and appendix to the Docusaurus site.

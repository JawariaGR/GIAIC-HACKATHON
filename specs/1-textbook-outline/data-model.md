# Data Model: Textbook Content Structure

**Date**: 2025-12-05

This document defines the conceptual structure for the content of the "Physical AI & Humanoid Robotics" textbook. It does not represent a software data model but rather the organization of the educational material.

## Core Entities

### 1. Module
A `Module` is the highest-level thematic grouping in the book.

-   **Fields**:
    -   `Title` (string): The name of the module (e.g., "Module 1: ROS 2 and Robotics Fundamentals").
    -   `Goal` (string): A brief description of the module's primary learning objective.
    -   `Chapters` (list of `Chapter`): An ordered list of chapters contained within the module.

### 2. Chapter
A `Chapter` represents a discrete unit of learning, typically corresponding to one week of study.

-   **Fields**:
    -   `Title` (string): The title of the chapter (e.g., "Chapter 2: ROS 2 Nodes, Topics, and Messages").
    -   `Introduction` (text): A brief overview of the chapter's content and relevance.
    -   `Learning Outcomes` (list of strings): A bulleted list of measurable skills a student will acquire.
    -   `Key Concepts` (list of strings): The core theoretical ideas presented in the chapter.
    -   `Main Text` (Markdown): The primary instructional content, including diagrams and code snippets.
    -   `Lab` (`Lab` object): The associated hands-on exercise for the chapter.
    -   `Summary` (text): A concise recap of the chapter's key takeaways.
    -   `Quiz` (`Quiz` object): A set of questions to test understanding.

### 3. Lab
A `Lab` is a hands-on, practical exercise to reinforce the concepts taught in a `Chapter`.

-   **Fields**:
    -   `Title` (string): The name of the lab (e.g., "Lab: Creating a ROS 2 Publisher/Subscriber").
    -   `Objective` (string): A clear statement of what the lab aims to achieve.
    -   `Prerequisite Skills` (list of strings): Knowledge required before starting the lab.
    -   `Instructions` (Markdown): Numbered, step-by-step instructions to complete the lab.
    -   `Code Listings` (list of code blocks): Complete, working code snippets required for the lab.
    -   `Expected Output` (text/image): A description or screenshot of the expected result.

### 4. Quiz
A `Quiz` is a short assessment to validate a student's comprehension of a `Chapter`.

-   **Fields**:
    -   `Questions` (list of `Question` objects): A list of questions.
    -   `Answers` (list of `Answer` objects): The corresponding answers and explanations.

## Relationships

-   A `Book` contains multiple `Modules`.
-   A `Module` contains multiple `Chapters`.
-   A `Chapter` has one `Lab` and one `Quiz`.

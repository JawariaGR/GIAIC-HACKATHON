# Research: Technology Versioning for "Physical AI & Humanoid Robotics"

**Date**: 2025-12-05

## 1. Decision: ROS 2 Version

### Decision
The textbook and all associated code examples will standardize on **ROS 2 Humble Hawksbill**.

### Rationale
-   **Long-Term Support (LTS)**: ROS 2 Humble is the latest LTS release, ensuring stability, security updates, and community support for the lifetime of the textbook. This is critical for a beginner-friendly educational resource, as it minimizes the risk of encountering breaking changes or deprecated features.
-   **Package Availability**: As an LTS release, Humble has the widest support for third-party packages and tools, which will be essential for later chapters.
-   **Target Platform**: It officially targets Ubuntu 22.04, which is also an LTS release, providing a stable and well-documented operating system foundation for students.

### Alternatives Considered
-   **ROS 2 Iron Irwini**: This is a more recent non-LTS release. While it has newer features, it has a shorter support lifespan and may be less stable. For a foundational textbook, the stability of an LTS release is preferable.
-   **Newer ROS 2 Releases**: Future releases would be too new and not have the same level of proven stability and community documentation required for a beginner's textbook.

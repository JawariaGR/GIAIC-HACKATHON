# Quickstart: Setting Up Your Robotics Development Environment

**Date**: 2025-12-05

Welcome to "Physical AI & Humanoid Robotics"! This guide will walk you through setting up the necessary software to follow along with the book's examples and labs.

## 1. Operating System: Ubuntu 22.04 LTS

For the best compatibility with the ROS 2 version we'll be using, it is strongly recommended to use **Ubuntu 22.04 LTS**.

-   **Option A (Recommended): Dual Boot or Bare Metal**: If you are able, installing Ubuntu 22.04 directly on your machine will provide the best performance for simulations.
-   **Option B: Virtual Machine (VM)**: You can use software like VirtualBox or VMware to install Ubuntu 22.04 inside your current operating system. Ensure you allocate sufficient resources (e.g., 4+ CPU cores, 8GB+ RAM, 50GB+ disk space).
-   **Option C: Docker**: For more advanced users, a Docker container can be used. A `Dockerfile` may be provided with the course materials.

## 2. Install ROS 2 Humble Hawksbill

Follow the official ROS 2 documentation to install ROS 2 Humble.

1.  **Set Locale**:
    ```bash
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    ```
2.  **Add ROS 2 APT Repository**:
    ```bash
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
3.  **Install ROS 2 Desktop**:
    We recommend the full desktop install to get RViz, Gazebo, and other tools.
    ```bash
    sudo apt update
    sudo apt install ros-humble-desktop
    ```
4.  **Source the Setup File**:
    Add this line to the end of your `~/.bashrc` file to make ROS 2 available in all your terminals.
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

## 3. Create Your ROS 2 Workspace

A workspace (`colcon_ws`) is where you will build and run your own ROS 2 packages.

1.  Create the directory:
    ```bash
    mkdir -p ~/ros2_ws/src
    ```
2.  Navigate into the workspace:
    ```bash
    cd ~/ros2_ws
    ```

## 4. Run the "Hello World" of ROS: Turtlesim

Let's test your installation.

1.  **Open a new terminal** and run the turtlesim simulation:
    ```bash
    ros2 run turtlesim turtlesim_node
    ```
    You should see a new window appear with a blue background and a turtle in the center.

2.  **Open a second terminal** and run the keyboard controller:
    ```bash
    ros2 run turtlesim turtle_teleop_key
    ```

3.  Click on the second terminal and use the arrow keys on your keyboard. You should see the turtle in the first window move around.

**Congratulations! Your ROS 2 environment is set up and working.** You are now ready to begin Chapter 1.

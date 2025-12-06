# Quickstart Guide: Physical AI & Humanoid Robotics

This quickstart guide provides a basic overview to get the Physical AI & Humanoid Robotics project up and running in a simulated environment.

## 1. Prerequisites

Before you begin, ensure you have the following installed and configured:

- **Ubuntu 22.04 LTS**: The recommended operating system for ROS 2 development.
- **ROS 2 Humble Hawksbill**: Follow the official [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html).
- **Python 3.10+**: Ensure Python and `pip` are correctly set up.
- **Git**: For cloning the repository.
- **Docker / NVIDIA Container Toolkit**: Recommended for easily managing NVIDIA Isaac dependencies.
- **NVIDIA GPU**: Required for NVIDIA Isaac components (Isaac Sim, Isaac ROS).

## 2. Setting up the Development Environment

### 2.1. Clone the Repository

```bash
git clone [REPOSITORY_URL]
cd physical-ai-humanoid-robotics
```

### 2.2. Initialize ROS 2 Workspace

```bash
mkdir -p robotics_ws/src
cd robotics_ws
vcs import src < [REPOSITORY_URL]/src/robotics_ws.repos # Example if using a single repo with submodules/vcs
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### 2.3. NVIDIA Isaac Setup

Follow the specific installation guides for:
- **NVIDIA Isaac Sim**: [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_basic.html)
- **NVIDIA Isaac ROS**: [Isaac ROS Documentation](https://docs.ros.org/en/isaac_ros/)

Consider using Docker containers for Isaac ROS to simplify dependency management.

## 3. Running the Simulation

### 3.1. Launch Humanoid in Gazebo (Example)

First, ensure your ROS 2 environment is sourced.

```bash
ros2 launch robot_description display.launch.py # To display URDF
ros2 launch robot_control gazebo_simulation.launch.py # To spawn in Gazebo
```

### 3.2. Launch Unity / Isaac Sim Visualization (Example)

Start the Unity Editor or Isaac Sim application and load the relevant scene/assets as per their documentation to connect to the ROS 2 environment.

## 4. Basic Interaction: Voice Command

Once the simulation is running and the VLA pipeline is active:

1. Run the Whisper node (example):
   ```bash
   ros2 run vla_pipeline whisper_node
   ```
2. Speak a command (e.g., "Robot, go to the table") into your microphone.
3. Observe the robot's behavior in the simulator.

## 5. Next Steps

- Explore individual ROS 2 packages within `robotics_ws/src/`.
- Refer to the Implementation Plan for detailed architectural decisions.
- Check the Feature Tasks section for implementation tasks.
# Navigation Pipeline Documentation

This document describes the navigation pipeline for the Physical AI & Humanoid Robotics project.

## Overview

The navigation pipeline enables the humanoid robot to understand navigation commands and autonomously navigate to specified locations in a simulated environment. The system processes navigation commands through a series of interconnected ROS 2 nodes that plan paths, execute navigation actions, and handle environmental perception.

## Architecture

The navigation pipeline consists of the following main components:

### 1. Command Processing
- **Input**: NavigationCommand messages
- **Processing**: Semantic understanding and target parsing
- **Output**: ActionPlan messages

### 2. Path Planning
- **Input**: Navigation targets and environmental data
- **Processing**: Path computation using Nav2
- **Output**: Planned Path messages

### 3. Action Execution
- **Input**: ActionPlan messages
- **Processing**: Navigation action execution
- **Output**: Navigation results

### 4. Environmental Perception
- **Input**: Sensor data (LiDAR, camera)
- **Processing**: VSLAM and environment mapping
- **Output**: Odometry and map data

## Data Flow

1. A navigation command is received by the Nav2 Planner Node
2. The command is parsed to extract the target location
3. A path is planned from the current position to the target
4. An ActionPlan is created and published
5. The Action Executor receives the plan and initiates navigation
6. The Nav2 Action Server executes the navigation goal
7. Sensor data is continuously processed for localization and mapping
8. Results are logged to the MCP server for analysis

## Key ROS 2 Messages

- `NavigationCommand.msg`: Defines navigation commands for the robot
- `ActionPlan.msg`: Represents a sequence of high-level tasks
- `NavigateToPose.action`: Action for commanding robot to navigate to a target pose

## Key ROS 2 Nodes

- `nav2_planner_node.py`: Handles navigation command processing and path planning
- `nav2_action_server.py`: Executes navigation goals using Nav2
- `action_executor.py`: Interprets action plans and triggers appropriate actions
- `vslam_node.py`: Provides localization and environment mapping

## MCP Server Integration

Navigation data is logged to the MCP server for versioning and analysis:
- Navigation commands received
- Planned paths
- Action plan creation
- Navigation execution results
- Performance metrics
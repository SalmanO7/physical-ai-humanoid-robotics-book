# ROS 2 Interface Definitions

This section describes the ROS 2 message (`.msg`), service (`.srv`), and action (`.action`) definitions that serve as the API contracts for the Physical AI & Humanoid Robotics system.

## Message Definitions

- **`robot_control/msg/JointState.msg`**: Defines the current state of robot joints (position, velocity, effort).
- **`perception/msg/ObjectDetection.msg`**: Defines detected objects in the environment (bounding box, class, confidence, pose).
- **`vla_pipeline/msg/VoiceCommandText.msg`**: Defines the text transcript of a voice command.
- **`vla_pipeline/msg/ActionPlan.msg`**: Defines a sequence of high-level actions.

## Service Definitions

- **`robot_control/srv/SetJointPosition.srv`**: Service to command specific joint positions.
  - Request: `string joint_name, float64 position`
  - Response: `bool success`
- **`perception/srv/DetectObjects.srv`**: Service to trigger object detection and return results.
  - Request: `bool enable_detection`
  - Response: `perception/msg/ObjectDetection[] detections`

## Action Definitions

- **`navigation/action/NavigateToPose.action`**: Action to command the robot to navigate to a target pose.
  - Goal: `geometry_msgs/PoseStamped target_pose`
  - Result: `bool success, string message`
  - Feedback: `float64 distance_remaining, float64 angle_remaining`
- **`manipulation/action/GraspObject.action`**: Action to command the robot to grasp an identified object.
  - Goal: `string object_id`
  - Result: `bool success, string message`
  - Feedback: `float64 progress`

These definitions serve as the API contracts between different components of the system, ensuring consistent communication patterns across the entire architecture.
#!/usr/bin/env python3
"""
Action Executor for Humanoid Robot VLA Pipeline

This node subscribes to ActionPlan messages and executes them by triggering
appropriate ROS 2 actions, including NavigateToPose for navigation tasks.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose as NavigateToPoseAction

from vla_pipeline.msg import ActionPlan
from vla_pipeline.msg import NavigationCommand


class ActionExecutor(Node):
    """
    Executes action plans by triggering appropriate ROS 2 actions.
    """

    def __init__(self):
        super().__init__('action_executor')

        # Create action client for navigation
        self.nav_action_client = ActionClient(
            self,
            NavigateToPoseAction,
            'navigate_to_pose'
        )

        # Subscriber for ActionPlan messages
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.action_plan_sub = self.create_subscription(
            ActionPlan,
            'action_plan',
            self.action_plan_callback,
            qos_profile
        )

        # Publisher for navigation commands (for logging/monitoring)
        self.nav_cmd_pub = self.create_publisher(
            NavigationCommand,
            'executed_navigation_command',
            qos_profile
        )

        # Wait for navigation action server to be available
        self.get_logger().info('Waiting for navigation action server...')
        self.nav_action_client.wait_for_server()
        self.get_logger().info('Navigation action server available')

        self.get_logger().info('Action Executor initialized')

    def action_plan_callback(self, msg):
        """
        Process incoming action plans and execute navigation tasks.
        """
        self.get_logger().info(f'Received action plan with {len(msg.task_list)} tasks')

        for i, task in enumerate(msg.task_list):
            self.get_logger().info(f'Processing task {i+1}: {task}')

            # Check if this is a navigation task
            if task.startswith('NAVIGATE_TO('):
                self.execute_navigation_task(task, msg)
            elif task.startswith('IDENTIFY_OBJECT('):
                self.execute_object_identification_task(task, msg)
            elif task.startswith('MANIPULATE_OBJECT('):
                self.execute_manipulation_task(task, msg)
            else:
                self.get_logger().warn(f'Unknown task type: {task}')

    def execute_navigation_task(self, task, action_plan):
        """
        Execute a navigation task by sending a NavigateToPose action goal.
        """
        self.get_logger().info(f'Executing navigation task: {task}')

        # Parse the navigation target from the task string
        # Expected format: NAVIGATE_TO(location_name) or NAVIGATE_TO(x,y,z,yaw)
        target = self.parse_navigation_target(task)

        if target is None:
            self.get_logger().error(f'Could not parse navigation target from: {task}')
            return

        # Create navigation goal
        goal_msg = NavigateToPoseAction.Goal()

        # Set target pose based on parsed target
        if isinstance(target, dict) and 'x' in target and 'y' in target:
            # Numeric coordinates
            goal_msg.pose = PoseStamped()
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.pose.position.x = target['x']
            goal_msg.pose.pose.position.y = target['y']
            goal_msg.pose.pose.position.z = target.get('z', 0.0)

            # Set orientation (yaw)
            import math
            yaw = target.get('yaw', 0.0)
            # Convert yaw to quaternion
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            goal_msg.pose.pose.orientation.w = cy
            goal_msg.pose.pose.orientation.z = sy
        else:
            # Named location - for now, use predefined coordinates
            # In a real implementation, this would look up coordinates from a map
            coords = self.get_coordinates_for_location(target)
            if coords:
                goal_msg.pose = PoseStamped()
                goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
                goal_msg.pose.header.frame_id = 'map'
                goal_msg.pose.pose.position.x = coords['x']
                goal_msg.pose.pose.position.y = coords['y']
                goal_msg.pose.pose.position.z = coords.get('z', 0.0)
                goal_msg.pose.pose.orientation.w = 1.0  # No rotation for simplicity

        # Send navigation goal
        self.get_logger().info(f'Sending navigation goal to ({goal_msg.pose.pose.position.x}, {goal_msg.pose.pose.position.y})')

        # Use async send goal (non-blocking)
        future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )

        # Add done callback to handle result
        future.add_done_callback(lambda future: self.navigation_result_callback(future, task))

    def parse_navigation_target(self, task):
        """
        Parse navigation target from task string.
        """
        import re

        # Try to parse coordinates: NAVIGATE_TO(x,y,z,yaw) or NAVIGATE_TO(x,y)
        coord_pattern = r'NAVIGATE_TO\(([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+)(?:,\s*([-+]?\d*\.?\d+))?(?:,\s*([-+]?\d*\.?\d+))?\)'
        coord_match = re.search(coord_pattern, task)

        if coord_match:
            x = float(coord_match.group(1))
            y = float(coord_match.group(2))
            z = float(coord_match.group(3)) if coord_match.group(3) else 0.0
            yaw = float(coord_match.group(4)) if coord_match.group(4) else 0.0

            return {'x': x, 'y': y, 'z': z, 'yaw': yaw}

        # Try to parse named location: NAVIGATE_TO(location_name)
        name_pattern = r'NAVIGATE_TO\((\w+)\)'
        name_match = re.search(name_pattern, task)

        if name_match:
            return name_match.group(1)

        return None

    def get_coordinates_for_location(self, location_name):
        """
        Get coordinates for a named location.
        In a real implementation, this would look up coordinates from a map.
        """
        # Predefined locations for demonstration
        locations = {
            'kitchen': {'x': 2.0, 'y': 1.0, 'z': 0.0},
            'living_room': {'x': -1.0, 'y': 2.0, 'z': 0.0},
            'bedroom': {'x': -2.0, 'y': -1.0, 'z': 0.0},
            'office': {'x': 1.0, 'y': -2.0, 'z': 0.0},
            'table': {'x': 0.5, 'y': 0.5, 'z': 0.0}
        }

        return locations.get(location_name.lower())

    def navigation_feedback_callback(self, feedback_msg):
        """
        Handle navigation feedback.
        """
        self.get_logger().debug(
            f'Navigation feedback: {feedback_msg.feedback.distance_remaining:.2f}m remaining'
        )

    def navigation_result_callback(self, future, original_task):
        """
        Handle navigation result.
        """
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Navigation goal was rejected')
                return

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(
                lambda future: self.final_navigation_result_callback(future, original_task)
            )
        except Exception as e:
            self.get_logger().error(f'Exception in navigation result callback: {e}')

    def final_navigation_result_callback(self, future, original_task):
        """
        Handle final navigation result.
        """
        try:
            result = future.result().result
            if result.result:
                self.get_logger().info(f'Navigation task completed successfully: {original_task}')
            else:
                self.get_logger().warn(f'Navigation task failed: {original_task} - {result.message}')
        except Exception as e:
            self.get_logger().error(f'Exception in final navigation result callback: {e}')

    def execute_object_identification_task(self, task, action_plan):
        """
        Execute an object identification task.
        """
        self.get_logger().info(f'Executing object identification task: {task}')
        # Implementation would go here

    def execute_manipulation_task(self, task, action_plan):
        """
        Execute a manipulation task.
        """
        self.get_logger().info(f'Executing manipulation task: {task}')
        # Implementation would go here


def main(args=None):
    rclpy.init(args=args)
    action_executor = ActionExecutor()

    try:
        rclpy.spin(action_executor)
    except KeyboardInterrupt:
        pass
    finally:
        action_executor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
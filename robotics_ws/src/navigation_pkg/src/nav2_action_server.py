#!/usr/bin/env python3
"""
Nav2 Action Server for Humanoid Robot Navigation

This node implements a ROS 2 action server for the NavigateToPose action,
which commands the robot to navigate to a target pose using Nav2.
"""

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose as NavigateToPoseAction
import time
import math


class Nav2ActionServer(Node):
    """
    Action server that handles NavigateToPose goals for the humanoid robot.
    """

    def __init__(self):
        super().__init__('nav2_action_server')

        # Create action server with a reentrant callback group for concurrent goals
        self.action_server = ActionServer(
            self,
            NavigateToPoseAction,
            'navigate_to_pose',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        # Publisher for current pose (for feedback simulation)
        self.pose_pub = self.create_publisher(PoseStamped, 'current_pose', 10)

        self.get_logger().info('Nav2 Action Server initialized')

    def goal_callback(self, goal_request):
        """
        Accept or reject a goal request.
        """
        self.get_logger().info('Received navigation goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Accept or reject a cancel request.
        """
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        Execute the navigation goal.
        """
        self.get_logger().info('Executing navigation goal')

        # Get target pose from goal
        target_pose = goal_handle.request.pose
        self.get_logger().info(f'Navigating to: ({target_pose.pose.position.x:.2f}, {target_pose.pose.position.y:.2f})')

        # Initialize feedback
        feedback_msg = NavigateToPoseAction.Feedback()
        result = NavigateToPoseAction.Result()

        # Simulate navigation progress
        start_time = self.get_clock().now()
        current_time = start_time
        total_time = 5.0  # seconds for simulation

        # Simulate robot movement toward target
        current_x = 0.0  # Starting position (would be actual pose in real implementation)
        current_y = 0.0

        # Calculate distance to target
        dx = target_pose.pose.position.x - current_x
        dy = target_pose.pose.position.y - current_y
        target_distance = math.sqrt(dx*dx + dy*dy)

        # Simulate navigation with progress updates
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.result = False
                result.message = 'Goal canceled'
                self.get_logger().info('Navigation goal canceled')
                return result

            # Update current position (simulated movement)
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            progress = min(elapsed / total_time, 1.0)

            current_x = current_x + dx * progress
            current_y = current_y + dy * progress

            # Calculate remaining distance
            remaining_dx = target_pose.pose.position.x - current_x
            remaining_dy = target_pose.pose.position.y - current_y
            distance_remaining = math.sqrt(remaining_dx*remaining_dx + remaining_dy*remaining_dy)

            # Calculate angle remaining (simplified)
            angle_remaining = math.atan2(remaining_dy, remaining_dx)

            # Update feedback
            feedback_msg.distance_remaining = distance_remaining
            feedback_msg.angle_remaining = angle_remaining

            # Publish current pose (for visualization)
            current_pose_msg = PoseStamped()
            current_pose_msg.header.stamp = self.get_clock().now().to_msg()
            current_pose_msg.header.frame_id = 'map'
            current_pose_msg.pose.position.x = current_x
            current_pose_msg.pose.position.y = current_y
            current_pose_msg.pose.position.z = 0.0
            current_pose_msg.pose.orientation.w = 1.0  # No rotation for simplicity

            self.pose_pub.publish(current_pose_msg)

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().debug(f'Progress: {progress*100:.1f}%, Distance: {distance_remaining:.2f}m')

            # Check if we've reached the goal (with tolerance)
            if distance_remaining < 0.1:  # 10cm tolerance
                goal_handle.succeed()
                result.result = True
                result.message = f'Reached goal in {(elapsed):.2f} seconds'
                self.get_logger().info(f'Navigation succeeded: {result.message}')
                return result

            # Check if goal has taken too long
            if elapsed > total_time * 2:  # Timeout after 2x expected time
                goal_handle.abort()
                result.result = False
                result.message = 'Navigation timeout'
                self.get_logger().info(f'Navigation failed: {result.message}')
                return result

            # Sleep briefly to simulate processing
            time.sleep(0.1)

        # If we get here, something went wrong
        goal_handle.abort()
        result.result = False
        result.message = 'Navigation failed due to interruption'
        self.get_logger().info(f'Navigation failed: {result.message}')
        return result


def main(args=None):
    rclpy.init(args=args)

    nav2_action_server = Nav2ActionServer()

    try:
        # Use a multi-threaded executor to handle multiple goals concurrently
        executor = MultiThreadedExecutor()
        executor.add_node(nav2_action_server)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nav2_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
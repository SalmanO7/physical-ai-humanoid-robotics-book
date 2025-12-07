#!/usr/bin/env python3
"""
Nav2 Planner Node for Humanoid Robot Navigation

This node interfaces with the Nav2 stack to provide path planning
capabilities for the humanoid robot in simulated environments.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from vla_pipeline.msg import NavigationCommand
from vla_pipeline.msg import ActionPlan

# Import MCP logger
from .mcp_logger import MCPServerLogger


class Nav2PlannerNode(Node):
    """
    Node that interfaces with Nav2 for path planning and navigation.
    """

    def __init__(self):
        super().__init__('nav2_planner_node')

        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.path_pub = self.create_publisher(Path, 'planned_path', qos_profile)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            qos_profile
        )
        self.status_pub = self.create_publisher(String, 'nav2_status', qos_profile)

        # Subscribers
        self.nav_cmd_sub = self.create_subscription(
            NavigationCommand,
            'navigation_command',
            self.navigation_command_callback,
            qos_profile
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            qos_profile
        )

        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',  # LiDAR data from Gazebo
            self.laser_scan_callback,
            qos_profile
        )

        # Timer for periodic updates
        self.timer = self.create_timer(1.0, self.update_planner_status)

        # Internal state
        self.current_map = None
        self.current_laser_data = None
        self.current_goal = None
        self.planned_path = None

        # MCP server logger
        try:
            self.mcp_logger = MCPServerLogger(server_url="http://localhost:8080")
            self.get_logger().info('MCP Server logger initialized')
        except Exception as e:
            self.mcp_logger = None
            self.get_logger().warn(f'Failed to initialize MCP Server logger: {e}')

        self.get_logger().info('Nav2 Planner Node initialized')

    def navigation_command_callback(self, msg):
        """
        Process navigation commands and plan paths.
        """
        self.get_logger().info(f'Received navigation command: {msg.command_text}')
        self.get_logger().info(f'Semantic meaning: {msg.semantic_meaning}')

        # Log the navigation command to MCP server
        if self.mcp_logger:
            log_success = self.mcp_logger.log_navigation_event(
                'navigation_command_received',
                {
                    'command_text': msg.command_text,
                    'semantic_meaning': msg.semantic_meaning,
                    'timestamp': msg.timestamp.sec + msg.timestamp.nanosec / 1e9
                }
            )
            if log_success:
                self.get_logger().debug('Navigation command logged to MCP server')
            else:
                self.get_logger().warn('Failed to log navigation command to MCP server')

        # Parse the navigation target from the command
        target_pose = self.parse_navigation_target(msg.semantic_meaning)

        if target_pose is None:
            self.get_logger().error(f'Could not parse navigation target from: {msg.semantic_meaning}')
            return

        # Plan path to target
        path = self.plan_path_to_target(target_pose)

        if path is not None:
            self.planned_path = path
            self.path_pub.publish(path)
            self.get_logger().info(f'Published planned path with {len(path.poses)} waypoints')

            # Log the planned path to MCP server
            if self.mcp_logger:
                path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
                log_success = self.mcp_logger.log_path_data(
                    path_points,
                    {
                        'command_text': msg.command_text,
                        'num_waypoints': len(path.poses),
                        'planning_algorithm': 'straight_line_simulation'
                    }
                )
                if log_success:
                    self.get_logger().debug('Planned path logged to MCP server')
                else:
                    self.get_logger().warn('Failed to log planned path to MCP server')

            # Create an action plan to execute the navigation
            self.create_and_publish_action_plan(target_pose, msg.command_text)
        else:
            self.get_logger().error('Failed to plan path to target')

    def parse_navigation_target(self, semantic_meaning):
        """
        Parse navigation target from semantic meaning.
        """
        import re

        # Look for coordinates in the semantic meaning
        coord_pattern = r'(\w+)\s+at\s+position\s+([-\+]?\d*\.?\d+)\s*,\s*([-\+]?\d*\.?\d+)'
        coord_match = re.search(coord_pattern, semantic_meaning)

        if coord_match:
            location_type = coord_match.group(1)
            x = float(coord_match.group(2))
            y = float(coord_match.group(3))

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            return pose

        # Look for named locations
        named_locations = {
            'kitchen': (2.0, 1.0),
            'living room': (0.0, 2.0),
            'bedroom': (-2.0, -1.0),
            'office': (1.0, -2.0),
            'table': (0.5, 0.5),
            'door': (-1.0, 0.0)
        }

        semantic_lower = semantic_meaning.lower()
        for location_name, (x, y) in named_locations.items():
            if location_name in semantic_lower:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'map'
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0

                return pose

        # If no specific location found, return a default pose
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 1.0
        pose.pose.position.y = 1.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        return pose

    def plan_path_to_target(self, target_pose):
        """
        Plan a path to the target pose.
        In a real implementation, this would interface with Nav2's planner.
        For simulation, we'll create a simple path.
        """
        self.get_logger().info(f'Planning path to target: ({target_pose.pose.position.x:.2f}, {target_pose.pose.position.y:.2f})')

        # In a real implementation, this would call Nav2's planner service
        # For simulation, we'll create a straight-line path with some waypoints
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        # For simulation, create a simple path (in a real implementation, this would come from Nav2)
        start_x, start_y = 0.0, 0.0  # Starting position
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y

        # Create waypoints along the path (straight line for simplicity)
        num_waypoints = 10
        for i in range(num_waypoints + 1):
            fraction = i / num_waypoints
            x = start_x + (target_x - start_x) * fraction
            y = start_y + (target_y - start_y) * fraction

            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            # Simple orientation pointing toward the target
            import math
            if i < num_waypoints:
                next_x = start_x + (target_x - start_x) * (i + 1) / num_waypoints
                next_y = start_y + (target_y - start_y) * (i + 1) / num_waypoints
                yaw = math.atan2(next_y - y, next_x - x)
                pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
                pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                pose_stamped.pose.orientation.w = 1.0

            path.poses.append(pose_stamped)

        self.get_logger().info(f'Planned path with {len(path.poses)} waypoints')
        return path

    def create_and_publish_action_plan(self, target_pose, original_command):
        """
        Create and publish an action plan based on the navigation target.
        """
        action_plan = ActionPlan()
        action_plan.timestamp = self.get_clock().now().to_msg()
        action_plan.generated_by = self.get_name()
        action_plan.execution_status = 'pending'

        # Create navigation task
        nav_task = f'NAVIGATE_TO({target_pose.pose.position.x:.2f},{target_pose.pose.position.y:.2f})'
        action_plan.task_list = [nav_task]
        action_plan.dependencies = []

        # Publish the action plan
        action_plan_pub = self.create_publisher(
            ActionPlan,
            'action_plan',
            10
        )
        action_plan_pub.publish(action_plan)
        self.get_logger().info(f'Published action plan: {nav_task}')

        # Log the action plan creation to MCP server
        if self.mcp_logger:
            log_success = self.mcp_logger.log_navigation_event(
                'action_plan_created',
                {
                    'original_command': original_command,
                    'target_x': target_pose.pose.position.x,
                    'target_y': target_pose.pose.position.y,
                    'task_list': action_plan.task_list,
                    'dependencies': action_plan.dependencies
                }
            )
            if log_success:
                self.get_logger().debug('Action plan creation logged to MCP server')
            else:
                self.get_logger().warn('Failed to log action plan creation to MCP server')

    def map_callback(self, msg):
        """
        Handle map updates from the navigation system.
        """
        self.current_map = msg
        self.get_logger().debug(f'Received map with resolution {msg.info.resolution}')

    def laser_scan_callback(self, msg):
        """
        Handle laser scan data for obstacle detection and path replanning.
        """
        self.current_laser_data = msg
        self.get_logger().debug(f'Received laser scan with {len(msg.ranges)} ranges')

        # Check for obstacles in the path and potentially replan
        if self.planned_path is not None:
            if self.detect_obstacles_in_path(msg):
                self.get_logger().warn('Obstacle detected in path, replanning...')
                # In a real implementation, we would replan the path here
                # For now, just log the event

    def detect_obstacles_in_path(self, laser_scan):
        """
        Detect if there are obstacles in the current planned path.
        """
        if self.planned_path is None or len(laser_scan.ranges) == 0:
            return False

        # Simple obstacle detection: check if any range reading is below threshold
        min_distance = min(laser_scan.ranges)
        obstacle_threshold = 0.5  # meters

        return min_distance < obstacle_threshold

    def update_planner_status(self):
        """
        Periodically update the planner status.
        """
        status_msg = String()
        status_msg.data = 'active'
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    nav2_planner_node = Nav2PlannerNode()

    try:
        rclpy.spin(nav2_planner_node)
    except KeyboardInterrupt:
        pass
    finally:
        nav2_planner_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
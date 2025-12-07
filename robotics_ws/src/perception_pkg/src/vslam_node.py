#!/usr/bin/env python3
"""
VSLAM Node for Humanoid Robot Perception

This node implements Visual Simultaneous Localization and Mapping (VSLAM)
using Isaac ROS for localization in the simulated environment.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np


class VSLAMNode(Node):
    """
    Visual SLAM implementation for humanoid robot localization and mapping.
    """

    def __init__(self):
        super().__init__('vslam_node')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'vslam/odometry', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'vslam/pose', 10)
        self.map_pub = self.create_publisher(MarkerArray, 'vslam/map', 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            'camera/camera_info',
            self.camera_info_callback,
            10
        )

        # LiDAR subscriber for environment mapping
        self.lidar_sub = self.create_subscription(
            LaserScan,
            'scan',  # LiDAR data from Gazebo
            self.lidar_callback,
            10
        )

        # Timer for processing loop
        self.timer = self.create_timer(0.1, self.process_vslam)

        # Internal state
        self.current_image = None
        self.camera_info = None
        self.lidar_data = None
        self.current_pose = np.eye(4)  # 4x4 identity matrix for pose
        self.map_points = []

        self.get_logger().info('VSLAM Node initialized')

    def image_callback(self, msg):
        """Process incoming camera images for VSLAM."""
        self.current_image = msg
        self.get_logger().debug(f'Received image: {msg.width}x{msg.height}')

    def camera_info_callback(self, msg):
        """Process camera calibration information."""
        self.camera_info = msg
        self.get_logger().debug('Received camera info')

    def lidar_callback(self, msg):
        """Process incoming LiDAR data for environment mapping."""
        self.lidar_data = msg
        self.get_logger().debug(f'Received LiDAR scan with {len(msg.ranges)} ranges')

        # Process LiDAR data for environment mapping
        self.process_lidar_for_mapping(msg)

    def process_lidar_for_mapping(self, lidar_msg):
        """Process LiDAR data to build environment map."""
        # Convert LiDAR ranges to Cartesian coordinates
        angle_min = lidar_msg.angle_min
        angle_increment = lidar_msg.angle_increment

        # Clear previous map points
        self.map_points = []

        # Convert each range reading to a point in space
        for i, range_val in enumerate(lidar_msg.ranges):
            if not (lidar_msg.range_min <= range_val <= lidar_msg.range_max):
                # Skip invalid range readings
                continue

            angle = angle_min + i * angle_increment

            # Convert to Cartesian coordinates (relative to robot)
            x = range_val * np.cos(angle)
            y = range_val * np.sin(angle)

            # Add to map points
            self.map_points.append((x, y))

        self.get_logger().debug(f'Processed {len(self.map_points)} map points from LiDAR data')

    def process_vslam(self):
        """Main VSLAM processing loop."""
        if self.current_image is None or self.camera_info is None:
            # Still process LiDAR data even if camera data is not available
            if self.lidar_data is not None:
                self.get_logger().debug('Processing LiDAR-only data')
                self.publish_map()
            return

        # Simulate VSLAM processing
        # In a real implementation, this would interface with Isaac ROS VSLAM
        self.get_logger().debug('Processing VSLAM')

        # Update pose based on simulated motion
        self.update_pose()

        # Publish odometry
        self.publish_odometry()

        # Publish pose
        self.publish_pose()

        # Publish map if we have LiDAR data
        if self.lidar_data is not None:
            self.publish_map()

    def update_pose(self):
        """Update the robot's estimated pose based on visual features."""
        # Simulate pose update (in real implementation, this would use feature tracking)
        dt = 0.1  # timer period
        # Simulate small movement for testing
        self.current_pose[0, 3] += 0.01  # x translation
        self.current_pose[1, 3] += 0.005  # y translation

    def publish_odometry(self):
        """Publish odometry information."""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set position
        odom_msg.pose.pose.position.x = self.current_pose[0, 3]
        odom_msg.pose.pose.position.y = self.current_pose[1, 3]
        odom_msg.pose.pose.position.z = self.current_pose[2, 3]

        # Simple orientation (for now, just identity)
        odom_msg.pose.pose.orientation.w = 1.0

        # Set velocities (simulated)
        odom_msg.twist.twist.linear.x = 0.1
        odom_msg.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom_msg)

    def publish_pose(self):
        """Publish pose information."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'

        # Set position
        pose_msg.pose.position.x = self.current_pose[0, 3]
        pose_msg.pose.position.y = self.current_pose[1, 3]
        pose_msg.pose.position.z = self.current_pose[2, 3]

        # Simple orientation
        pose_msg.pose.orientation.w = 1.0

        self.pose_pub.publish(pose_msg)

    def publish_map(self):
        """Publish the environment map based on LiDAR data."""
        if not self.map_points:
            return

        marker_array = MarkerArray()

        # Create markers for each map point
        for i, (x, y) in enumerate(self.map_points):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'  # or 'odom' depending on your coordinate frame
            marker.ns = 'lidar_map'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Position (transform from robot frame to map frame if needed)
            marker.pose.position.x = x  # This would need transformation in a real implementation
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            # Scale
            marker.scale.x = 0.05  # 5cm spheres
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            # Color (blue for LiDAR points)
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.8  # 80% alpha

            marker_array.markers.append(marker)

        self.map_pub.publish(marker_array)
        self.get_logger().debug(f'Published map with {len(marker_array.markers)} points')


def main(args=None):
    rclpy.init(args=args)
    vslam_node = VSLAMNode()

    try:
        rclpy.spin(vslam_node)
    except KeyboardInterrupt:
        pass
    finally:
        vslam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
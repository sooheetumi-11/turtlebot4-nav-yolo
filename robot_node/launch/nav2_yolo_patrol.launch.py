# SPDX-License-Identifier: Apache-2.0
"""Run the three-goal Nav2 patrol alongside camera-based YOLO detection."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_topic', default_value='/camera/image',
            description='ROS Image topic published by the TurtleBot 4 camera'),
        DeclareLaunchArgument(
            'output_topic', default_value='/yolo/detections_image',
            description='Annotated image topic published by the detector'),
        Node(
            package='robot_node', executable='nav2_yolo_patrol',
            name='nav2_yolo_patrol', output='screen',
            parameters=[{
                'camera_topic': LaunchConfiguration('camera_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
            }]),
    ])

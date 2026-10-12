"""Standalone coverage planner service: ros2 launch coverage_planner_ros coverage_planner.launch.py [config_file:=...]"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    params = {}
    config_file = LaunchConfiguration('config_file').perform(context)
    if config_file:
        params['config_file'] = config_file
    return [Node(package='coverage_planner_ros', executable='coverage_planner', name='coverage_planner',
                 output='screen', parameters=[params])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value='',
                              description='Planner YAML; empty = coverage_planner_core default_planner.yaml'),
        OpaqueFunction(function=_setup),
    ])

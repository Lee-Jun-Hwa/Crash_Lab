#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
	use_sim_time = LaunchConfiguration('use_sim_time', default='false')
	cartographer_prefix = get_package_share_directory('pill_cartographer')
	tutorial_ros2_motor_prefix = get_package_share_directory('tutorial_ros2_motor')
	pill_description_prefix = get_package_share_directory('pill_description')
	ydlidar_prefix = get_package_share_directory('ydlidar_ros2_driver')



	resolution = LaunchConfiguration('resolution', default='0.05')
	publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')




	rviz_config_dir = os.path.join(
		get_package_share_directory('cartographer'),
		'rviz', 'cartographer.rviz')
	
	return LaunchDescription([
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([cartographer_prefix, '/launch','/cartographer.launch.py']),
			launch_arguments={'use_rviz': 'true'}.items()
		),
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([pill_description_prefix,'/launch','/tetra_description.launch.py']),
			launch_arguments={'use_rviz': 'false'}.items()
		),
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([ydlidar_prefix, '/launch','/ydlidar_launch.py']),
    	),
		Node(
			package='tutorial_ros2_motor',
			executable='test_motor_node',
			name='test_motor_node',
		)
	])

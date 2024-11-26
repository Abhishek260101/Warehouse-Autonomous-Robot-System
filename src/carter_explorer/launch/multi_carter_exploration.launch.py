from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'carter_explorer'
    
    return LaunchDescription([
        # Static transforms for LIDAR frames - using new style arguments
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_lidar_carter1',
            parameters=[{
                'frame_id': 'carter1/base_link',
                'child_frame_id': 'carter1/front_2d_lidar',
                'translation.x': 0.0,
                'translation.y': 0.0,
                'translation.z': 0.0,
                'rotation.x': 0.0,
                'rotation.y': 0.0,
                'rotation.z': 0.0,
                'rotation.w': 1.0,
            }]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_lidar_carter2',
            parameters=[{
                'frame_id': 'carter2/base_link',
                'child_frame_id': 'carter2/front_2d_lidar',
                'translation.x': 0.0,
                'translation.y': 0.0,
                'translation.z': 0.0,
                'rotation.x': 0.0,
                'rotation.y': 0.0,
                'rotation.z': 0.0,
                'rotation.w': 1.0,
            }]
        ),

        # SLAM nodes for both robots with adjusted parameters
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox_carter1',
            namespace='carter1',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'odom_frame': 'carter1/odom',
                'base_frame': 'carter1/base_link',
                'map_frame': 'carter1/map',
                'scan_topic': '/carter1/front_2d_lidar/scan',
                'mode': 'mapping',
                'use_scan_matching': True,
                'use_scan_bagging': True,
                'resolution': 0.05,
                'max_laser_range': 20.0,
                
                # Message handling parameters
                'transform_timeout': 1.0,
                'tf_buffer_duration': 30.0,
                'max_queue_size': 10,
                'minimum_time_interval': 0.2,
                'stack_size_to_use': 40000000,
                'scan_buffer_size': 5,
                
                # Update intervals
                'map_update_interval': 2.0,
                'map_publish_interval': 2.0,
                'transform_publish_period': 0.05,
                
                # Processing parameters
                'enable_interactive_mode': False,
                'loop_search_maximum_distance': 5.0,
                'minimum_travel_distance': 0.1,
                'minimum_travel_heading': 0.1,
                
                # Advanced parameters
                'laser_max_range': 20.0,
                'laser_min_range': 0.1,
                'laser_missing_measurement_ray_tracing': False,
                'progress_rate': 10.0
            }]
        ),
        
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox_carter2',
            namespace='carter2',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'odom_frame': 'carter2/odom',
                'base_frame': 'carter2/base_link',
                'map_frame': 'carter2/map',
                'scan_topic': '/carter2/front_2d_lidar/scan',
                'mode': 'mapping',
                'use_scan_matching': True,
                'use_scan_bagging': True,
                'resolution': 0.05,
                'max_laser_range': 20.0,
                
                # Message handling parameters
                'transform_timeout': 1.0,
                'tf_buffer_duration': 30.0,
                'max_queue_size': 10,
                'minimum_time_interval': 0.2,
                'stack_size_to_use': 40000000,
                'scan_buffer_size': 5,
                
                # Update intervals
                'map_update_interval': 2.0,
                'map_publish_interval': 2.0,
                'transform_publish_period': 0.05,
                
                # Processing parameters
                'enable_interactive_mode': False,
                'loop_search_maximum_distance': 5.0,
                'minimum_travel_distance': 0.1,
                'minimum_travel_heading': 0.1,
                
                # Advanced parameters
                'laser_max_range': 20.0,
                'laser_min_range': 0.1,
                'laser_missing_measurement_ray_tracing': False,
                'progress_rate': 10.0
            }]
        ),
        
        # Frontier exploration node
        Node(
            package=pkg_name,
            executable='carter_explorer',
            name='carter_explorer',
            output='screen',
            parameters=[{
                'min_frontier_size': 10,
                'exploration_timeout': 300.0,
                'max_linear_speed': 0.5,
                'max_angular_speed': 1.0,
                'use_sim_time': True
            }]
        ),
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    robot_name = LaunchConfiguration('robot_name')
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='carter1',
        description='Name of the robot'
    )

    # SLAM Toolbox node
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'max_laser_range': 20.0,
            'map_update_interval': 1.0,
            'resolution': 0.05,
            'base_frame': 'carter1/base_link',
            'odom_frame': 'carter1/odom',
            'map_frame': 'map',
            'scan_topic': '/carter1/front_2d_lidar/scan'
        }]
    )

    # Frontier Explorer node
    explorer_node = Node(
        package='carter_explorer',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'min_frontier_size': 10,
            'exploration_timeout': 300.0,
            'linear_velocity': 0.3,
            'angular_velocity': 0.5,
            'goal_tolerance': 0.5
        }]
    )

    return LaunchDescription([
        robot_name_arg,
        slam_node,
        explorer_node
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # TF Tree setup
    tf_tree = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'carter1/odom', 'map']
    )

    # SLAM Toolbox node with optimized parameters
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
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
            'scan_topic': '/carter1/front_2d_lidar/scan',
            'transform_timeout': 0.5,
            'scan_queue_size': 25,           # Increased queue size
            'map_start_pose': [0.0, 0.0, 0.0],
            'minimum_time_interval': 0.1,    # Process scans more frequently
            'enable_scan_matching': True,
            'use_scan_barycenter': True,
            'process_near_pose': True,       # Process scans near current pose
            'use_multithread_processing': True  # Enable multithreading
        }]
    )

    # Frontier Explorer node
    explorer_node = Node(
        package='carter_explorer',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'robot_name': 'carter1',
            'min_frontier_size': 10,
            'exploration_timeout': 300.0,
            'linear_velocity': 0.2,
            'angular_velocity': 0.3,
            'goal_tolerance': 0.5,
            'map_update_tolerance': 5.0,
            'rotation_duration': 12.0,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        tf_tree,
        slam_node,
        explorer_node
    ])
import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    robot_urdf = get_package_share_directory("robot_urdf")
    robot_controller = get_package_share_directory("robot_controller")
    robot_urdf_prefix = get_package_prefix("robot_urdf")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(robot_urdf, "urdf", "robot_urdf.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    model_path = os.path.join(robot_urdf, "models")
    model_path += pathsep + os.path.join(robot_urdf_prefix, "share")
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")
        )
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    # Delete entity before spawning
    delete_entity_cmd = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/delete_entity', 'gazebo_msgs/DeleteEntity', '{"name": "robot_urdf"}'],
        output='screen'
    )

    spawn_robot = Node(
        package="gazebo_ros", 
        executable="spawn_entity.py",
        arguments=["-entity", "robot_urdf",
                   "-topic", "robot_description",
                   "-x", "0", "-y", "0", "-z", "0.1"],  # Adjust spawn position as needed
        output="screen"
    )

    controller_params_file = os.path.join(robot_controller, "config", "robot_controller.yaml")

    # Load controllers
    load_controllers = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'simple_velocity_controller'],
        output='screen'
    )

    return LaunchDescription([
        env_var,
        model_arg,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_controllers],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_controllers,
                on_exit=[load_velocity_controller],
            )
        ),
    ])
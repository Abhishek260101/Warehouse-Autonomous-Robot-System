import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, GroupAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import UnlessCondition, IfCondition

def generate_launch_description():
    # Packages and directories
    robot_urdf = get_package_share_directory("robot_urdf")
    robot_controller = get_package_share_directory("robot_controller")
    robot_urdf_prefix = get_package_prefix("robot_urdf")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    # Launch configuration
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(robot_urdf, "urdf", "robot_urdf.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    # Set Gazebo model path
    model_path = os.path.join(robot_urdf, "models")
    model_path += pathsep + os.path.join(robot_urdf_prefix, "share")
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    # Robot description
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Gazebo
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

    # Spawn robot
    spawn_robot = Node(
        package="gazebo_ros", 
        executable="spawn_entity.py",
        arguments=["-entity", "robot_urdf",
                   "-topic", "robot_description",
                   "-x", "0", "-y", "0", "-z", "0.1"],
        output="screen"
    )

    # Load controllers
    controller_params_file = os.path.join(robot_controller, "config", "robot_controller.yaml")

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description},
                    controller_params_file,
                    {"use_sim_time": True}],
        output="screen",
    )

    # New arguments from reference file
    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="True",
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.16",
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.304",
    )
    
    use_simple_controller = LaunchConfiguration("use_simple_controller")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_controller", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(use_simple_controller),
    )

    # Simple controller group
    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["simple_velocity_controller", "--controller-manager", "/controller_manager"]
            ),
            Node(
                package="robot_controller",
                executable="simple_controller",
                parameters=[
                    {"wheel_radius": wheel_radius,
                     "wheel_separation": wheel_separation}],
            ),
        ]
    )

    # Delay loading of controllers
    delay_controller_manager = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[controller_manager],
        )
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[wheel_controller_spawner, simple_controller],
        )
    )

    return LaunchDescription([
        env_var,
        model_arg,
        use_simple_controller_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot,
        delay_controller_manager,
        delay_joint_state_broadcaster,
        delay_controllers
    ])
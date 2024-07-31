import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro

pkg_name = "self_driving_bicycle"
pkg_share_directory = get_package_share_directory(pkg_name)


def generate_launch_description():

    slam_params_file = LaunchConfiguration("slam_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    slam_params_file_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(
            pkg_share_directory, "config", "mapper_params_online_async.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for the slam_toolbox node",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )

    pkg_path = os.path.join(pkg_share_directory)
    xacro_file = os.path.join(pkg_path, "urdf", "bicycle_drive.xacro.urdf")
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    params = {"robot_description": robot_description_raw, "use_sim_time": True}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    node_joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        arguments=[xacro_file],
    )

    rviz_config = os.path.join(pkg_path, "config", "config.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                ),
            ]
        ),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "bicycle"],
        output="screen",
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_bicycle_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "bicycle_controller",
        ],
        output="screen",
    )

    static_transform_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--yaw",
            "0",
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            "odom",
            "--child-frame-id",
            "base_link",
        ],
    )

    odom_broadcaster = Node(
        package=pkg_name, executable="odom_broadcaster", name="odom_broadcaster"
    )

    slam_node = Node(
        parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
    )

    return LaunchDescription(
        [
            rviz,
            # node_joint_state_publisher_gui,
            node_robot_state_publisher,
            # static_transform_base_link,
            odom_broadcaster,
            gazebo,
            spawn_entity,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_bicycle_controller],
                )
            ),
            use_sim_time_arg,
            slam_params_file_arg,
            slam_node,
        ]
    )

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler

from launch.substitutions import Command, PathJoinSubstitution
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node

import xacro

pkg_name = "self_driving_bicycle_hw"
pkg_share_directory = get_package_share_directory(pkg_name)


def generate_launch_description():

    pkg_path = os.path.join(pkg_share_directory)
    xacro_file = os.path.join(pkg_path, "urdf", "bicycle_drive.xacro.urdf")
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    robot_description_params = {
        "robot_description": robot_description_raw,
        "use_sim_time": False,
    }

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description_params],
        remappings=[
            ("/bicycle_controller/reference_unstamped", "/cmd_vel"),
        ],
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

    controller_params = PathJoinSubstitution(
        [
            pkg_share_directory,
            "config",
            "bicycle_drive_controller.yaml",
        ]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description_params, controller_params],
        output="both",
        remappings=[
            ("/bicycle_controller/tf_odometry", "/tf"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    bicycle_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "bicycle_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz],
        )
    )

    delay_bicycle_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[bicycle_controller],
            )
        )
    )

    return LaunchDescription(
        [
            controller_manager,
            node_robot_state_publisher,
            # node_joint_state_publisher_gui,
            joint_state_broadcaster_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_bicycle_controller_spawner_after_joint_state_broadcaster_spawner,
        ]
    )

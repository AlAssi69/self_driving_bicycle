import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro

pkg_name = "self_driving_bicycle"
pkg_share_directory = get_package_share_directory(pkg_name)


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
    )

    pkg_path = os.path.join(pkg_share_directory)

    xacro_file = os.path.join(pkg_path, "urdf", "bicycle_drive.xacro.urdf")

    # Method 1
    # doc = xacro.parse(open(xacro_file))
    # xacro.process_doc(doc)
    # robot_description_raw = doc.toxml()

    # Method 2
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    params = {"robot_description": robot_description_raw}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
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

    load_tricycle_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "tricycle_controller",
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_path, "config", "config.rviz"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=spawn_entity,
            #         on_exit=[load_joint_state_broadcaster],
            #     )
            # ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=load_joint_state_broadcaster,
            #         on_exit=[load_tricycle_controller],
            #     )
            # ),
            # gazebo,
            rviz,
            node_robot_state_publisher,
            # spawn_entity,
        ]
    )

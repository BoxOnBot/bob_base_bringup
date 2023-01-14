# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("bob_base_bringup"), "rviz", "bob.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("bob_description"), "urdf", "bob.urdf.xacro"]
            ),
            " ",
            "simulated:='false'",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Get controller configs
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("bob_description"),
            "config",
            "bob_odrive_controller.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    # Robot state publisherss
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawn controllers

    left_wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["left_wheel_joint_velocity_controller", "-c", "/controller_manager"],
    )

    right_wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["right_wheel_joint_velocity_controller", "-c", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diffbot_base_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
                    robot_controller_spawner, 
                ],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_controller_spawner,
                on_exit=[
                    rviz_node,
                ],
            )
        ),
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource([
#                PathJoinSubstitution([
#                    FindPackageShare('bob_base_bringup'),
#                    'launch/bob_realsense_v2.launch.py'
#                ])
#            ])
#        )
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource([
#                PathJoinSubstitution([
#                    FindPackageShare('bob_base_bringup'),
#                    'launch/bob_monocular.launch.py'
#                ])
#            ])
#        )
    ])

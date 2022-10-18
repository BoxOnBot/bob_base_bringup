# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import launch
from launch_ros.actions import Node


def generate_launch_description():
    realsense_camera_node_front = Node(
        package='realsense2_camera',
        namespace="bob_front_stereo_camera",
        node_executable='realsense2_camera_node',
        parameters=[{
                'camera_name': 'bob_front_stereo_camera',
                'serial_no': '_934222071202',
                'infra_height': 360,
                'infra_width': 640,
                'enable_color': False,
                'enable_depth': False,
                'stereo_module.emitter_enabled': False,
                'infra_fps': 90.0
        }]
    )
    realsense_camera_node_back = Node(
        package='realsense2_camera',
        namespace="bob_back_stereo_camera",
        node_executable='realsense2_camera_node',
        parameters=[{
                'camera_name': 'bob_back_stereo_camera',
                'serial_no': '_935322071447',
                'infra_height': 360,
                'infra_width': 640,
                'enable_color': False,
                'enable_depth': False,
                'stereo_module.emitter_enabled': False,
                'infra_fps': 90.0
        }]
    )

    return launch.LaunchDescription([
        realsense_camera_node_front,
        realsense_camera_node_back
    ])

# Copyright 2018 Lucas Walter
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Lucas Walter nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

# Camera 1
bob_cam_1_param_path = PathJoinSubstitution(
    [
        FindPackageShare("bob_base_bringup"),
        "config",
        "cam_1",
        "params.yaml"
    ]
)

bob_cam_node_1 = Node(
    package="usb_cam",
    executable="usb_cam_node_exe",
    name="bob_cam_node_1",
    namespace="bob_cam_1",
    output="log",
    parameters=[bob_cam_1_param_path]
)

# Camera 2
bob_cam_2_param_path = PathJoinSubstitution(
    [
        FindPackageShare("bob_base_bringup"),
        "config",
        "cam_2",
        "params.yaml"
    ]
)

bob_cam_node_2 = Node(
    package="usb_cam",
    executable="usb_cam_node_exe",
    name="bob_cam_node_2",
    namespace="bob_cam_2",
    output="log",
    parameters=[bob_cam_2_param_path]
)

# Camera 3
bob_cam_3_param_path = PathJoinSubstitution(
    [
        FindPackageShare("bob_base_bringup"),
        "config",
        "cam_3",
        "params.yaml"
    ]
)

bob_cam_node_3 = Node(
    package="usb_cam",
    executable="usb_cam_node_exe",
    name="bob_cam_node_3",
    namespace="bob_cam_3",
    output="log",
    parameters=[bob_cam_3_param_path]
)

# Camera 4
bob_cam_4_param_path = PathJoinSubstitution(
    [
        FindPackageShare("bob_base_bringup"),
        "config",
        "cam_4",
        "params.yaml"
    ]
)

bob_cam_node_4 = Node(
    package="usb_cam",
    executable="usb_cam_node_exe",
    name="bob_cam_node_4",
    namespace="bob_cam_4",
    output="log",
    parameters=[bob_cam_4_param_path]
)


def generate_launch_description():
    return LaunchDescription([
        bob_cam_node_1,
        bob_cam_node_2,
        bob_cam_node_3,
        bob_cam_node_4
    ])

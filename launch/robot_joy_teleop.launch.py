# Copyright 2022 Factor Robotics
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
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("cps_loki_description"),
                    "urdf",
                    "odrive_diffbot.urdf.xacro"
                ]
            ),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("cps_loki_bringup"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "-c", "/controller_manager"],
    )

    joystick_spawner = Node(
        package="joy",
        executable="joy_node"
    )

    teleop_spawner = Node(
        package="rmp220_teleop",
        executable="rmp220_teleop",
        #remappings=[('/diffbot_base_controller/cmd_vel_unstamped','/cmd_vel_joy')]
        remappings=[('/cmd_vel','/diffbot_base_controller/cmd_vel_unstamped')]
    )

    cam_node = Node(
        package="ros2_cam_openCV",
        executable="cam_node"
    )

    lidar_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lsx10.yaml')         
    lidar_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[lidar_dir],
    )

    twist_mux_params = os.path.join(get_package_share_directory('cps_loki_bringup'),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': False}],
            remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')]
        )

    return LaunchDescription([
        #control_node,
        #robot_state_pub_node,
        #joint_state_broadcaster_spawner,
        #robot_controller_spawner,
        joystick_spawner,
        teleop_spawner,
        #cam_node,
        #lidar_node,
        #twist_mux
    ])

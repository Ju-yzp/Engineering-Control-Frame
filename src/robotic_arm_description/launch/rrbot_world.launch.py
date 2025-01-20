# Copyright 2021 Open Robotics (2021)
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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command
import xacro

def generate_launch_description():
    robot_name_in_model ="Staubli"
    package_name='robotic_arm_description'
    urdf_name = "robot_include.urdf.xacro"
    
    ld = LaunchDescription()
    
    pkg_share = FindPackageShare(package='robotic_arm_description').find('robotic_arm_description')
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')
    
    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch the robot
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description','-entity', 'robotic_arm_description'], output='screen')
    
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )

    arm_broad_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"]
    )
    ld.add_action(start_gazebo_cmd)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(joint_broad_spawner)
    ld.add_action(arm_broad_spawner)

    return ld

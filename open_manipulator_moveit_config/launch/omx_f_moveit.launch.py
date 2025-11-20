#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
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
#
# Author: Woojin Wie

## 이 코드는 rivz2를 실행하고 로봇팔을 로드하며 경로계획등을 할 수 있게 함
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


# launch argements 선언
def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [ 
        # launch파일 실행할때 Rviz 실행 여부 -> 디폴트 true
        DeclareLaunchArgument(
            'start_rviz', default_value='true', description='Whether to execute rviz2'
        ),
        # simulation 시간 사용할지
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Whether to use simulation time',
        ),
        # moveit 저장 db 경로
        DeclareLaunchArgument(
            'warehouse_sqlite_path',
            default_value=os.path.expanduser('~/.ros/warehouse_ros.sqlite'),
            description='Path where the warehouse database should be stored',
        ),
        # SRDF 정보 퍼블리시 여부
        DeclareLaunchArgument(
            'publish_robot_description_semantic',
            default_value='true',
            description='Whether to publish robot description semantic',
        ),
    ]

    # 위 설정 적용
    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')
    warehouse_sqlite_path = LaunchConfiguration('warehouse_sqlite_path')
    publish_robot_description_semantic = LaunchConfiguration('publish_robot_description_semantic')

    '''
    | 파일                        | 의미                                                 |
    | --------------------------- | ---------------------------------------             |
    | **.srdf**                   | 로봇의 그룹/EEF/충돌 제외 설정 등 추가설정             |
    | **joint_limits.yaml**       | 각 joint의 최대 속도/가속도 제한                      |
    | **moveit_controllers.yaml** | MoveIt이 사용할 controller 설정 (팔 움직임 명령 전달)  |
    | **kinematics.yaml**         | IK solver(KDL 등) 설정                               |
    '''
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='omx_f', package_name='open_manipulator_moveit_config')
        .robot_description_semantic(
            str(Path('config') / 'omx_f' / 'omx_f.srdf'))
        .joint_limits(str(Path('config') / 'omx_f' / 'joint_limits.yaml'))
        .trajectory_execution(
            str(Path('config') / 'omx_f' / 'moveit_controllers.yaml'))
        .robot_description_kinematics(
            str(Path('config') / 'omx_f' / 'kinematics.yaml'))
        .to_moveit_configs()
    )

    # MoveIt에서 사용하는 DB 설정 -> 경로계획 결과를 저장하는 db : 없어도 될듯?
    warehouse_ros_config = {
        'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'warehouse_host': warehouse_sqlite_path,
    }

    # moveit의 핵심 movegroup 설정 위에서 설정했던 설정값을 가지고 movegroup을 만듦
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                'use_sim_time': use_sim,
                'publish_robot_description_semantic': publish_robot_description_semantic,
            },
        ],
    )

    # RViz2 실행
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('open_manipulator_moveit_config'), 'config', 'moveit.rviz']
    )
    rviz_node = Node(
        package='rviz2',
        condition=IfCondition(start_rviz),
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {
                'use_sim_time': use_sim,
            },
        ],
    )

    # return LaunchDescription(declared_arguments + [move_group_node, rviz_node])
    return LaunchDescription(
        declared_arguments
        + [
            move_group_node,
            rviz_node,
        ]
    )
#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Joep Tool, HyunGyu Kim

import os
import xml.etree.ElementTree as ET
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

CONFIG_PATH = '/home/zenos/ws/tb3_formation/config/formation_config.yml'
def get_config_para(param):
    '''
    加载全局config文件中的参数，用绝对路径
    '''
    with open(CONFIG_PATH, 'r',encoding='UTF-8') as file:
        config = yaml.safe_load(file)

    return config[param]

# config here
TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

NUM_OF_ROBOTS = get_config_para('NUM_OF_ROBOTS')
INIT_POSE = get_config_para('INIT_POSE')

def generate_launch_description():
    namespace = 'robot'
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )
    save_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'tmp'
    )
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'empty_world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd_list = []

    for count in range(NUM_OF_ROBOTS):
        robot_state_publisher_cmd_list.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'frame_prefix': f'{namespace}_{count+1}'
                    }.items()
            )
        )

    spawn_turtlebot_cmd_list = []

    for count in range(NUM_OF_ROBOTS):
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        for odom_frame_tag in root.iter('odometry_frame'):
            odom_frame_tag.text = f'{namespace}_{count+1}/odom'
        for base_frame_tag in root.iter('robot_base_frame'):
            base_frame_tag.text = f'{namespace}_{count+1}/base_footprint'
        for scan_frame_tag in root.iter('frame_name'):
            scan_frame_tag.text = f'{namespace}_{count+1}/base_scan'
        urdf_modified = ET.tostring(tree.getroot(), encoding='unicode')
        urdf_modified = '<?xml version="1.0" ?>\n'+urdf_modified
        with open(f'{save_path}{count+1}.sdf', 'w', encoding='UTF-8') as file:
            file.write(urdf_modified)

        spawn_turtlebot_cmd_list.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir, 'multi_spawn_turtlebot3.launch.py')
                ),
                launch_arguments={
                        'x_pose': str(INIT_POSE[count][0]),
                        'y_pose': str(INIT_POSE[count][1]),
                        'yaw_pose': str(INIT_POSE[count][2]),
                        'robot_name': f'{TURTLEBOT3_MODEL}_{count+1}',
                        'namespace': f'{namespace}_{count+1}',
                        'sdf_path': f'{save_path}{count+1}.sdf'
                }.items()
            )
        )

    ld = LaunchDescription()
    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(RegisterEventHandler(
        OnShutdown(
            on_shutdown=lambda event,
            context: [os.remove(f'{save_path}{count+1}.sdf') \
                      for count in range(NUM_OF_ROBOTS)]
        )
    ))
    for count, spawn_turtlebot_cmd in enumerate(spawn_turtlebot_cmd_list, start=1):
        ld.add_action(GroupAction([PushRosNamespace(f'{namespace}_{count}'),
                                  robot_state_publisher_cmd_list[count-1],
                                  spawn_turtlebot_cmd]))

    return ld

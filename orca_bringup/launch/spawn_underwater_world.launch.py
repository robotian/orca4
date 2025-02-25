#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2022 Clyde McQueen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Launch a simulation.

Includes Gazebo, ArduSub, RViz, mavros, all ROS nodes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Load Gazebo world model and run ArduSub plugin

def generate_launch_description():
    orca_description_dir = get_package_share_directory('orca_description')        

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")        

    # Gazebo.
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-v3 -s -r "   # -v: option is for verbose level
            + os.path.join(orca_description_dir, 'worlds', 'sand_empty.world')
        }.items(),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-v3 -g"}.items(),
    )

    ld = LaunchDescription()
    ld.add_action(gz_sim_server)
    ld.add_action(gz_sim_gui)


    # world_file = os.path.join(orca_description_dir, 'worlds', 'sand_empty.world')
    # declare_gzclient_cmd = DeclareLaunchArgument(
    #     'gzclient',
    #     default_value='True',
    #     description='Launch Gazebo UI?')
    

    # start_gzsim_cmd =  ExecuteProcess(
    #     cmd=['gz', 'sim', '-v', '3', '-r', world_file],
    #     output='screen',
    #     condition=IfCondition(LaunchConfiguration('gzclient')))

    # # Create the launch description and populate
    # ld = LaunchDescription()
    # ld.add_action(declare_gzclient_cmd)
    # ld.add_action(start_gzsim_cmd)


    

    return ld




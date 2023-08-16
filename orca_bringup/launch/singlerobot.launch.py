#!/usr/bin/env python3

# for a single vehicle simulation

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    orca_bringup_dir  = get_package_share_directory('orca_bringup')    
    orca_description_dir = get_package_share_directory('orca_description')

    # Ardusub specific parameters
    ardusub_params_file = os.path.join(orca_bringup_dir, 'cfg', 'sub.parm')

    # Mavros parameters: need to specify 'system_id' and communication address for each robot
    mavros_params_file = os.path.join(orca_bringup_dir, 'params', 'sim_mavros_params.yaml')


    orca_params_file = os.path.join(orca_bringup_dir, 'params', 'sim_orca_params.yaml')


    # the current world file includes the specific model names
    world_file = os.path.join(orca_description_dir, 'worlds', 'sand.world')

    

    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    slam = LaunchConfiguration("slam")

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace")

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace", default_value="false", description="Whether to apply a namespace to the navigation stack"
    )
    declare_slam_cmd = DeclareLaunchArgument("slam", default_value="False", description="Whether run a SLAM")







    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)

    return ld







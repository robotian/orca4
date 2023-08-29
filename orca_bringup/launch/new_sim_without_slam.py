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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')   
    orca_description_dir = get_package_share_directory('orca_description')    

    set_use_sim_time = SetParameter(name='use_sim_time', value=False)

    # Start Gazebo with default underwater world
    spawn_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(orca_bringup_dir, 'launch', 'spawn_underwater_world.launch.py'))                       
        )
    
    
    
    robots = [
        {'name': 'rov1', 'x_pose': 0.0, 'y_pose': 0.0, 'z_pose': 0.0,
                           'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0, 'instance':'-I0','home':'33.810313,-118.39386700000001,0.0,270.0'},
        # {'name': 'rov2', 'x_pose': 3.0, 'y_pose': 3.0, 'z_pose': 0.0,
        #                    'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0, 'instance':'-I1','home':'33.810311,-118.39386700000001,0.0,270.0'},
        # {'name': 'rov3', 'x_pose': -3.0, 'y_pose': 3.0, 'z_pose': 0.0,
        #                    'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0, 'console':'-I2','home':'33.810311,-118.39386700000001,0.0,270.0'}                           
        ]
        
    
    instances_cmds = []

    for robot in robots:        
        ardusub_rov_params_file = os.path.join(orca_bringup_dir, 'cfg',  f"sub_{robot['name']}.parm")
        mavros_params_file = os.path.join(orca_bringup_dir, 'params',  f"sim_mavros_params_{robot['name']}.yaml")
        orca_params_file = os.path.join(orca_bringup_dir, 'params', f"sim_orca_params_{robot['name']}.yaml")

        # Launch ArduSub w/ SIM_JSON (wipe eeprom w/ -w)
        # ardusub must be on the $PATH, see src/orca4/setup.bash
        start_ardusub_rov_cmd = ExecuteProcess(
            cmd=['ardusub', '-S', '-w', '-M', 'JSON', '--defaults', ardusub_rov_params_file,
                robot['instance'], '--home', robot['home']], 
            output='screen')
        
        # start_ardusub_rov_cmd = ExecuteProcess(
        #     cmd=['sim_vehicle.py','-C','-v','ArduSub','--sim-address=localhost','-I0','-w','--model','JSON','--no-mavproxy','--udp','--add-param-file',ardusub_rov_params_file],
        #     output='screen')
        # "mavproxy.py" "--out" "127.0.0.1:14550" "--master" "tcp:127.0.0.1:5760" "--sitl" "127.0.0.1:5501" "--map" "--console"
        # "mavproxy.py" "--out" "127.0.0.1:14560" "--master" "tcp:127.0.0.1:5770" "--sitl" "127.0.0.1:5511" "--map" "--console"
        
        # spawn rov sdf model
        sdf_filepath = os.path.join(orca_description_dir, 'models', f"tethered_{robot['name']}", 'model_no_cable.sdf')        
        opt_str = ['sdf_filename:"{name}"'.format(name=sdf_filepath),
                   'name:"{rov_name}"'.format(rov_name=f"tethered_{robot['name']}"), 
                   'pose:{position:{',
                   'x:', str(robot['x_pose']),
                   ',y:', str(robot['y_pose']),
                   ',z:', str(robot['z_pose']),r'},',
                   'orientation:{',
                   'x:', str(robot['x']),
                   ',y:', str(robot['y']),
                   ',z:', str(robot['z']),
                   ',w:', str(robot['w']),r'}}']
        start_spawn_rov1_cmd =  ExecuteProcess(
            cmd=['gz', 'service', '-s', '/world/sand/create', '--reqtype', 'gz.msgs.EntityFactory',
                 '--reptype','gz.msgs.Boolean','--timeout','1000','--req',opt_str],                 
            output='screen')
        

        # start_gz_brdg_cmd = Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     namespace=robot['name'],
        #     arguments=[
        #         f"/model/tethered_{robot['name']}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
        #     output='screen')
        
        # Gazebo Bridge.
        start_gz_brdg_cmd = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=robot['name'],
            parameters=[
                {
                    "config_file": os.path.join(
                        orca_bringup_dir, "cfg", f"{robot['name']}_bridge.yaml" 
                    ),
                    "qos_overrides./tf_static.publisher.durability": "transient_local",
                }
            ],
            output="screen",
        )
        
        nav2_bt_file = os.path.join(orca_bringup_dir, 'behavior_trees', 'orca4_bt.xml')
        nav2_params_file = os.path.join(orca_bringup_dir, 'params', f"nav2_{robot['name']}_params.yaml")
        
        # get_package_share_directory('orb_slam2_ros') will fail if orb_slam2_ros isn't installed
        orb_voc_file = os.path.join('install', 'orb_slam2_ros', 'share', 'orb_slam2_ros',
                                'orb_slam2', 'Vocabulary', 'ORBvoc.txt')
        
        # Rewrite to add the full path
        # The rewriter will only rewrite existing keys
        configured_nav2_params = RewrittenYaml(
            source_file=nav2_params_file,
            param_rewrites={
                'default_nav_to_pose_bt_xml': nav2_bt_file,
            },
            convert_types=True)
        
        set_env_var_cmd = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

        # Translate messages MAV <-> ROS
        start_mavros_cmd = Node(
            package='mavros',
            executable='mavros_node',
            namespace = f"{robot['name']}/mavros",
            output='screen',
            # mavros_node is actually many nodes, so we can't override the name
            # name='mavros_node',            
            parameters=[
                mavros_params_file,
                # {
                #     "fcu_url": "tcp://localhost",
                #     "gcs_url": "udp://@localhost:14550",    
                #     "tgt_system" : 1,
                #     # "tgt_component": 1,
                #     # "heartbeat_rate": 1.0,
                #     # "publish_sim_time": "false",                        
                # }
            ]
            )

        




        # bringup_cmd = IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(orca_bringup_dir, 'launch', 'new_bringup_rov.py')),
        #     launch_arguments={
        #         'namespace':robot['name'],
        #         # 'base': LaunchConfiguration('base'),
        #         # 'mavros': LaunchConfiguration('mavros'),
        #         'mavros_params_file': mavros_params_file,
        #         # 'nav': LaunchConfiguration('nav'),
        #         'orca_params_file': orca_params_file,
        #         # 'slam': LaunchConfiguration('slam')
        #         }.items())

        instances_cmds.append(set_use_sim_time)
        instances_cmds.append(start_ardusub_rov_cmd)
        instances_cmds.append(start_spawn_rov1_cmd)
        instances_cmds.append(start_gz_brdg_cmd)
        # instances_cmds.append(bringup_cmd)
        instances_cmds.append(set_env_var_cmd)
        # instances_cmds.append(start_mavros_cmd)

    
    

    




  
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(spawn_world_cmd)

    for inst in instances_cmds:
        ld.add_action(inst)

    return ld


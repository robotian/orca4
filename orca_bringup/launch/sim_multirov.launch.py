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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')
    orca_description_dir = get_package_share_directory('orca_description')

    # robots = [
    #     {'name': 'rov1', 'x_pose': 0.0, 'y_pose': 0.5, 'z_pose': 0.01,
    #                        'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
    #     {'name': 'rov2', 'x_pose': 0.0, 'y_pose': -0.5, 'z_pose': 0.01,
    #                        'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}]

    robots = [
        {'name': 'rov1', 'x_pose': 0.0, 'y_pose': 0.5, 'z_pose': 0.01,
                           'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                           'console':'-I0'}]
    

    # rosbag2_record_qos_file = os.path.join(orca_bringup_dir, 'params', 'rosbag2_record_qos.yaml')
    rviz_file = os.path.join(orca_bringup_dir, 'cfg', 'sim_multi_launch.rviz')
    world_file = os.path.join(orca_description_dir, 'worlds', 'sand_empty.world')
    ardusub_rov1_params_file = os.path.join(orca_bringup_dir, 'cfg',  f"sub_{robots[0]['name']}.parm")

    sim_left_ini = os.path.join(orca_bringup_dir, 'cfg', 'sim_left.ini')
    sim_right_ini = os.path.join(orca_bringup_dir, 'cfg', 'sim_right.ini')

    logger = LaunchConfiguration("log_level")

    declare_loglevel_cmd = DeclareLaunchArgument(
            "log_level",
            default_value=["debug"],
            description="Logging level",
    )

    # declare_namespace_cmd = DeclareLaunchArgument(
    #     'namespace',
    #     default_value='',
    #     description='Top-level namespace')

    declare_ardusub_cmd = DeclareLaunchArgument(
        'ardusub',
        default_value='True', 
        description='Launch ArduSUB with SIM_JSON?')
    
    # declare_bag_cmd = DeclareLaunchArgument(
    #     'bag',
    #     default_value='False',
    #     description='Bag interesting topics?')
    
    declare_base_cmd = DeclareLaunchArgument(
        'base',
        default_value='True',
        description='Launch base controller?')
    
    declare_gzclient_cmd = DeclareLaunchArgument(
        'gzclient',
        default_value='True',
        description='Launch Gazebo UI?')
    
    declare_mavros_cmd = DeclareLaunchArgument(
        'mavros',
        default_value='True',
        description='Launch mavros?')
    
    declare_nav_cmd = DeclareLaunchArgument(
        'nav',
        default_value='True',
        description='Launch navigation?')
    
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Launch rviz?')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Launch SLAM?')
    
    start_rviz_cmd = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')))
    

    # Launch ArduSub w/ SIM_JSON (wipe eeprom w/ -w)
        # ardusub must be on the $PATH, see src/orca4/setup.bash
    start_ardusub_rov1_cmd = ExecuteProcess(
        cmd=['ardusub', '-S', '-w', '-M', 'JSON', '--defaults', ardusub_rov1_params_file,
            '-I0', '--home', '33.810313,-118.39386700000001,0.0,270.0'],  # need to modify 
        output='screen',
        condition=IfCondition(LaunchConfiguration('ardusub')))
    
    # start_ardusub_rov2_cmd =    ExecuteProcess(
    #     cmd=['ardusub', '-S', '-w', '-M', 'JSON', '--defaults', os.path.join(orca_bringup_dir, 'cfg', f"sub_{robots[1]['name']}.parm") ,
    #         '-I1', '--home', '33.810314,-118.39386700000002, 0.0, 270.0'],  # need to modify 
    #     output='screen',
    #     condition=IfCondition(LaunchConfiguration('ardusub')))
    
    # Launch Gazebo Sim
        # gz must be on the $PATH
        # libArduPilotPlugin.so must be on the GZ_SIM_SYSTEM_PLUGIN_PATH
    start_gzsim_cmd =  ExecuteProcess(
        cmd=['gz', 'sim', '-v', '3', '-r', world_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gzclient')))
    
        # Spawn ROV1
    start_spawn_rov1_cmd =  ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/sand/create', '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype','gz.msgs.Boolean','--timeout','1000','--req','sdf_filename:"/home/robotian/ros2_ws/src/orca4/orca_description/models/tethered_rov1/model.sdf", name:"tethered_rov1",pose:{position:{x:0,y:0,z:0},orientation:{x:0,y:0,z:0,w:1}}'],
        output='screen')
    
    start_spawn_rov2_cmd =  ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/sand/create', '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype','gz.msgs.Boolean','--timeout','1000','--req','sdf_filename:"/home/robotian/ros2_ws/src/orca4/orca_description/models/tethered_rov2/model.sdf", name:"tethered_rov2",pose:{position:{x:4,y:4,z:0},orientation:{x:0,y:0,z:0,w:1}}'],
        output='screen')
    

    instances_cmds = []

    # mavros_params_file = os.path.join(orca_bringup_dir, 'params',  'sim_mavros_params_rov1.yaml')

    for robot in robots:        
        mavros_params_file = os.path.join(orca_bringup_dir, 'params',  f"sim_mavros_params_{robot['name']}.yaml")        
        orca_params_file = os.path.join(orca_bringup_dir, 'params', f"sim_orca_params_{robot['name']}.yaml")
        # orca_params_file = os.path.join(orca_bringup_dir, 'params', 'sim_orca_params_rov1.yaml')
        group = GroupAction([
            # Get images from Gazebo Sim to ROS
            Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['stereo_left', 'stereo_right'],
            namespace=robot['name'],
            output='screen'            
            ),

            # Gazebo Sim doesn't publish camera info, so do that here
            Node(
            package='orca_base',
            executable='camera_info_publisher',
            name='left_info_publisher',
            output='screen',
            namespace=robot['name'],
            parameters=[{
                'camera_info_url': 'file://' + sim_left_ini,
                'camera_name': 'stereo_left',
                'frame_id': 'stereo_left_frame',
                'timer_period_ms': 50}],
            remappings=[('camera_info', 'stereo_left/camera_info')]),

            # Gazebo Sim doesn't publish camera info, so do that here
            Node(
            package='orca_base',
            executable='camera_info_publisher',
            name='right_info_publisher',
            output='screen',
            namespace=robot['name'],
            parameters=[{
                'camera_info_url': 'file://' + sim_right_ini,
                'camera_name': 'stereo_right',
                'frame_id': 'stereo_right_frame',
                'timer_period_ms': 50}],
            remappings=[('camera_info', 'stereo_right/camera_info')]),

            Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=robot['name'],
            arguments=[
                f"/model/tethered_{robot['name']}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
            output='screen'),

            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(orca_bringup_dir, 'launch', 'bringup_rov.py')),
            launch_arguments={
                'namespace':robot['name'],
                'base': LaunchConfiguration('base'),
                'mavros': LaunchConfiguration('mavros'),
                'mavros_params_file': mavros_params_file,
                'nav': LaunchConfiguration('nav'),
                'orca_params_file': orca_params_file,
                'slam': LaunchConfiguration('slam')}.items())
        ])
        instances_cmds.append(group)
  
    # Create the launch description and populate
    ld = LaunchDescription()

    # ld.add_action(declare_loglevel_cmd)
    ld.add_action(declare_ardusub_cmd)
    # ld.add_action(declare_bag_cmd)
    ld.add_action(declare_base_cmd)
    ld.add_action(declare_gzclient_cmd)
    ld.add_action(declare_mavros_cmd)
    ld.add_action(declare_nav_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_slam_cmd)

    ld.add_action(start_gzsim_cmd)
    ld.add_action(start_rviz_cmd)

    ld.add_action(start_ardusub_rov1_cmd)
    # ld.add_action(start_ardusub_rov2_cmd)
    
    ld.add_action(start_spawn_rov1_cmd)
    # ld.add_action(start_spawn_rov2_cmd)

    # ld.add_action(start_image_bridge_node_cmd)
    # ld.add_action(start_left_camera_info_pub_node_cmd)
    # ld.add_action(start_right_camera_info_pub_node_cmd)
    # ld.add_action(start_param_bridge_node_cmd)
    # ld.add_action(include_bringup_cmd)


    for simulation_instance_cmd in instances_cmds:
        ld.add_action(simulation_instance_cmd)


    return ld





    # return LaunchDescription([
    #     DeclareLaunchArgument(
    #         'ardusub',
    #         default_value='True',
    #         description='Launch ArduSUB with SIM_JSON?'
    #     ),

    #     DeclareLaunchArgument(
    #         'bag',
    #         default_value='False',
    #         description='Bag interesting topics?',
    #     ),

    #     DeclareLaunchArgument(
    #         'base',
    #         default_value='True',
    #         description='Launch base controller?',
    #     ),

    #     DeclareLaunchArgument(
    #         'gzclient',
    #         default_value='True',
    #         description='Launch Gazebo UI?'
    #     ),

    #     DeclareLaunchArgument(
    #         'mavros',
    #         default_value='True',
    #         description='Launch mavros?',
    #     ),

    #     DeclareLaunchArgument(
    #         'nav',
    #         default_value='True',
    #         description='Launch navigation?',
    #     ),

    #     DeclareLaunchArgument(
    #         'rviz',
    #         default_value='True',
    #         description='Launch rviz?',
    #     ),

    #     DeclareLaunchArgument(
    #         'slam',
    #         default_value='True',
    #         description='Launch SLAM?',
    #     ),

    #     # Bag useful topics
    #     # ExecuteProcess(
    #     #     cmd=[
    #     #         'ros2', 'bag', 'record',
    #     #         '--qos-profile-overrides-path', rosbag2_record_qos_file,
    #     #         '--include-hidden-topics',
    #     #         '/cmd_vel',
    #     #         '/mavros/local_position/pose',
    #     #         '/mavros/rc/override',
    #     #         '/mavros/setpoint_position/global',
    #     #         '/mavros/state',
    #     #         '/mavros/vision_pose/pose',
    #     #         '/model/orca4/odometry',
    #     #         '/motion',
    #     #         '/odom',
    #     #         '/orb_slam2_stereo_node/pose',
    #     #         '/orb_slam2_stereo_node/status',
    #     #         '/pid_z',
    #     #         '/rosout',
    #     #         '/tf',
    #     #         '/tf_static',
    #     #     ],
    #     #     output='screen',
    #     #     condition=IfCondition(LaunchConfiguration('bag')),
    #     # ),

    #     # Launch rviz
    #     ExecuteProcess(
    #         cmd=['rviz2', '-d', rviz_file],
    #         output='screen',
    #         condition=IfCondition(LaunchConfiguration('rviz')),
    #     ),

    #     # Launch ArduSub w/ SIM_JSON (wipe eeprom w/ -w)
    #     # ardusub must be on the $PATH, see src/orca4/setup.bash
    #     ExecuteProcess(
    #         cmd=['ardusub', '-S', '-w', '-M', 'JSON', '--defaults', ardusub_params_file,
    #              '-I0', '--home', '33.810313,-118.39386700000001, 0.0, 270.0'],  # need to modify 
    #         output='screen',
    #         condition=IfCondition(LaunchConfiguration('ardusub')), 
    #     ),

    #     # Launch Gazebo Sim
    #     # gz must be on the $PATH
    #     # libArduPilotPlugin.so must be on the GZ_SIM_SYSTEM_PLUGIN_PATH
    #     ExecuteProcess(
    #         cmd=['gz', 'sim', '-v', '3', '-r', world_file],
    #         output='screen',
    #         condition=IfCondition(LaunchConfiguration('gzclient')),
    #     ),

    #     # Launch Gazebo Sim server-only
    #     ExecuteProcess(
    #         cmd=['gz', 'sim', '-v', '3', '-r', '-s', world_file],
    #         output='screen',
    #         condition=UnlessCondition(LaunchConfiguration('gzclient')),
    #     ),

    #     # Spawn ROV1
    #     ExecuteProcess(
    #         cmd=['gz', 'service', '-s', '/world/sand/create', '--reqtype', 'gz.msgs.EntityFactory',
    #              '--reptype','gz.msgs.Boolean','--timeout','1000','--req','sdf_filename:"/home/robotian/ros2_ws/src/orca4/orca_description/models/tethered_rov1/model.sdf", name:"tethered_rov1",pose:{position:{x:0,y:0,z:0},orientation:{x:0,y:0,z:0,w:1}}'],
    #         output='screen',
    #         # condition=IfCondition(LaunchConfiguration('gzclient')),
    #     ),

    #     # Get images from Gazebo Sim to ROS
    #     Node(
    #         package='ros_gz_image',
    #         executable='image_bridge',
    #         arguments=['stereo_left', 'stereo_right'],
    #         output='screen',
    #     ),

    #     # Gazebo Sim doesn't publish camera info, so do that here
    #     Node(
    #         package='orca_base',
    #         executable='camera_info_publisher',
    #         name='left_info_publisher',
    #         output='screen',
    #         parameters=[{
    #             'camera_info_url': 'file://' + sim_left_ini,
    #             'camera_name': 'stereo_left',
    #             'frame_id': 'stereo_left_frame',
    #             'timer_period_ms': 50,
    #         }],
    #         remappings=[
    #             ('/camera_info', '/stereo_left/camera_info'),
    #         ],
    #     ),

    #     Node(
    #         package='orca_base',
    #         executable='camera_info_publisher',
    #         name='right_info_publisher',
    #         output='screen',
    #         parameters=[{
    #             'camera_info_url': 'file://' + sim_right_ini,
    #             'camera_name': 'stereo_right',
    #             'frame_id': 'stereo_right_frame',
    #             'timer_period_ms': 50,
    #         }],
    #         remappings=[
    #             ('/camera_info', '/stereo_right/camera_info'),
    #         ],
    #     ),

    #     # Publish ground truth pose from Ignition Gazebo
    #     Node(
    #         package='ros_gz_bridge',
    #         executable='parameter_bridge',
    #         arguments=[
    #             '/model/tethered_rov1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
    #         ],
    #         output='screen'
    #     ),

    #     # Bring up Orca and Nav2 nodes
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(orca_bringup_dir, 'launch', 'bringup.py')),
    #         launch_arguments={
    #             'base': LaunchConfiguration('base'),
    #             'mavros': LaunchConfiguration('mavros'),
    #             'mavros_params_file': mavros_params_file,
    #             'nav': LaunchConfiguration('nav'),
    #             'orca_params_file': orca_params_file,
    #             'slam': LaunchConfiguration('slam'),
    #         }.items(),
    #     ),
    # ])


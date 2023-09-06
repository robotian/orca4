#!/usr/bin/env bash

# Modify this for your environment

# Add results of ArduSub build
export PATH=$HOME/ros2_ws/src/ardupilot/build/sitl/bin:$PATH

# Optional: add autotest to the PATH, helpful for running sim_vehicle.py
export PATH=$HOME/ros2_ws/src/ardupilot/Tools/autotest:$PATH

# Add results of colcon build
source $HOME/orca_ws/install/setup.bash

# Add ardupilot_gazebo plugin
#export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ros2_ws/src/ardupilot_gazebo/build:$Z_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ros2_ws/build/ardupilot_gazebo:$Z_SIM_SYSTEM_PLUGIN_PATH

# Optional: add ardupilot_gazebo models and worlds
export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/src/ardupilot_gazebo/models:$HOME/ros2_ws/src//ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# Add bluerov2_ignition models and worlds
export GZ_SIM_RESOURCE_PATH=$HOME/orca_ws/src/bluerov2_ignition/models:$HOME/orca_ws/src/bluerov2_ignition/worlds:$GZ_SIM_RESOURCE_PATH

# Add orca4 models and worlds
export GZ_SIM_RESOURCE_PATH=$HOME/orca_ws/src/orca4/orca_description/models:$HOME/orca_ws/src/orca4/orca_description/worlds:$GZ_SIM_RESOURCE_PATH

# Build ros_gz on the humble branch for Gazebo Garden
export GZ_VERSION=garden

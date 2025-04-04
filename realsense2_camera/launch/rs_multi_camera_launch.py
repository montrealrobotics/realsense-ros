# Copyright 2023 Intel Corporation. All Rights Reserved.
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

# DESCRIPTION #
# ----------- #
# Use this launch file to launch 2 devices.
# The Parameters available for definition in the command line for each camera are described in rs_launch.configurable_parameters
# For each device, the parameter name was changed to include an index.
# For example: to set camera_name for device1 set parameter camera_name1.
# command line example:
# ros2 launch realsense2_camera rs_multi_camera_launch.py camera_name1:=D400 device_type2:=l5. device_type1:=d4..

"""Launch realsense2_camera node."""
import copy
from launch import LaunchDescription, LaunchContext
import launch_ros.actions
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys
from launch.conditions import IfCondition
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import rs_launch

local_parameters = [{'name': 'camera_name1', 'default': 'camera1', 'description': 'camera1 unique name'},
                    {'name': 'camera_name2', 'default': 'camera2', 'description': 'camera2 unique name'},
                    {'name': 'camera_namespace1', 'default': 'camera1', 'description': 'camera1 namespace'},
                    {'name': 'camera_namespace2', 'default': 'camera2', 'description': 'camera2 namespace'},
                    {'name': 'static_transform', 'default': 'false', 'description': 'bool for launching static transform'},
                    ]

def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params

def launch_static_transform_publisher_node(context : LaunchContext):
    camera_tf1 = None
    camera_tf2 = None

    # Determine frame names based on device types
    if context.launch_configurations['device_type1'] == 't265':
        camera_tf1 = context.launch_configurations['camera_name1'] + '_pose_frame'
    else:
        camera_tf2 = context.launch_configurations['camera_name1'] + '_link'

    if context.launch_configurations['device_type2'] == 't265':
        # Only set if not already set
        if camera_tf1 is None:
            camera_tf1 = context.launch_configurations['camera_name2'] + '_pose_frame'
    else:
        # Only set if not already set
        if camera_tf2 is None:
            camera_tf2 = context.launch_configurations['camera_name2'] + '_link'
    node = launch_ros.actions.Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            condition=IfCondition(LaunchConfiguration('static_transform')),
                arguments=[
                    "--x", "0",
                    "--y", "0.2",
                    "--z", "0",
                    "--roll", "0",
                    "--pitch", "0",
                    "--yaw", "0",
                    "--frame-id", camera_tf1,
                    "--child-frame-id", camera_tf2]
                )
    return [node]

def generate_launch_description():
    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch.configurable_parameters, '2')
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params1) +
        rs_launch.declare_configurable_parameters(params2) +
        [
        OpaqueFunction(function=rs_launch.launch_setup,
                       kwargs = {'params'           : set_configurable_parameters(params1),
                                 'param_name_suffix': '1'}),
        OpaqueFunction(function=rs_launch.launch_setup,
                       kwargs = {'params'           : set_configurable_parameters(params2),
                                 'param_name_suffix': '2'}),
        OpaqueFunction(function=launch_static_transform_publisher_node)
    ])

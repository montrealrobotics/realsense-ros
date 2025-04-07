# Copyright (c) 2018 Intel Corporation
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

"""Launch realsense2_camera node without rviz2."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import rs_launch

local_parameters = [{'name': 'camera_name1', 'default': 'camera', 'description': 'camera unique name'},
                    {'name': 'device_type1', 'default': 'd4.', 'description': 'choose device by type'},
                    {'name': 'camera_name2', 'default': 'camera_t265', 'description': 'camera unique name'},
                    {'name': 'device_type2', 'default': 't265', 'description': 'choose device by type'},
                    {'name': 'enable_fisheye12', 'default': 'false', 'description': 'enable fisheye'},
                    {'name': 'enable_fisheye22', 'default': 'false', 'description': 'enable fisheye'},
                    {'name': 'enable_pose2', 'default': 'true', 'description': 'enable pose stream'},
                    {'name': 'align_depth.enable1', 'default': 'true', 'description': 'enable align depth filter'},
                    {'name': 'rgb_camera.color_profile1',    'default': "640, 480, 30", 'description': 'format'},
                    {'name': 'depth_module.depth_profile1',    'default': "640, 480, 30", 'description': 'format'},
                    {'name': 'static_transform', 'default': 'false', 'description': 'bool for launching static transform'},
                    {'name': 'point_cloud.enable1', 'default': 'true', 'description': 'enable pointcloud'},
                   ]

def generate_launch_description():
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) + 
        [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_multi_camera_launch.py']),
            launch_arguments=rs_launch.set_configurable_parameters(local_parameters).items(),
        ),
    ])

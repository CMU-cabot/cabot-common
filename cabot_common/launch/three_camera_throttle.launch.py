#!/usr/bin/env python3
# Copyright (c) 2025  IBM Corporation
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


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():
    output = {'stderr': {'log'}}
    use_sim_time = LaunchConfiguration('use_sim_time')
    output_hz = LaunchConfiguration('output_hz')
    camera1 = LaunchConfiguration('camera1')
    camera2 = LaunchConfiguration('camera2')
    camera3 = LaunchConfiguration('camera3')

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("cabot_common")])),

        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('output_hz', default_value='1.0'),
        DeclareLaunchArgument('camera1', default_value='/camera1'),
        DeclareLaunchArgument('camera2', default_value='/camera2'),
        DeclareLaunchArgument('camera3', default_value='/camera3'),

        SetParameter('use_sim_time', use_sim_time),

        Node(
            package='cabot_common',
            executable='three_camera_throttle_node',
            name='three_camera_throttle',
            parameters=[
                    {
                        'output_hz': output_hz,
                        'in1.image': [camera1, '/image_raw/compressed'],
                        'in1.info': [camera1, '/camera_info'],
                        'in2.image': [camera2, '/image_raw/compressed'],
                        'in2.info': [camera2, '/camera_info'],
                        'in3.image': [camera3, '/image_raw/compressed'],
                        'in3.info': [camera3, '/camera_info'],
                        'out1.image': [camera1, '/throttled/image_raw/compressed'],
                        'out1.info': [camera1, '/throttled/camera_info'],
                        'out2.image': [camera2, '/throttled/image_raw/compressed'],
                        'out2.info': [camera2, '/throttled/camera_info'],
                        'out3.image': [camera3, '/throttled/image_raw/compressed'],
                        'out3.info': [camera3, '/throttled/camera_info'],
                    },
            ],
            output=output,
        ),
    ])

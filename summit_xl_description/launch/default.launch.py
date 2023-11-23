# Copyright (c) 2023, Robotnik Automation S.L.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Robotnik Automation S.L.L. nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Robotnik Automation S.L.L. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

from robotnik_common.launch import RewrittenYaml

def read_params(ld : launch.LaunchDescription):
  environment = launch.substitutions.LaunchConfiguration('environment')
  robot_description_file = launch.substitutions.LaunchConfiguration('robot_description_file')
  robot_description_path = launch.substitutions.LaunchConfiguration('robot_description_path')
  robot_id = launch.substitutions.LaunchConfiguration('robot_id')
  robot_ns = launch.substitutions.LaunchConfiguration('robot_ns')

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='environment',
    description='Read parameters from environment variables',
    choices=['true', 'false'],
    default_value='true',
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='robot_description_file',
    description='Name of the file containing the robot description',
    default_value='summit_xl.urdf.xacro',
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='robot_description_path',
    description='Path to the file containing the robot description',
    default_value=[launch_ros.substitutions.FindPackageShare('summit_xl_description'), '/robots/', robot_description_file]
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='robot_id',
    description='Id of the robot',
    default_value='robot',
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='robot_ns',
    description='Namespace of the robot',
    default_value=robot_id,
  ))

  ret = {}

  if environment == 'false':
    ret = {
      'robot_description_path': robot_description_path,
      'robot_id': robot_id,
      'robot_ns': robot_ns,
    }
  else:
    if 'ROBOT_DESCRIPTION_PATH' in os.environ:
      ret['robot_description_path'] = os.environ['ROBOT_DESCRIPTION_PATH']
    elif 'ROBOT_DESCRIPTION_PACKAGE' in os.environ and 'ROBOT_DESCRIPTION_FILE' in os.environ:
      ret['robot_description_path'] = [get_package_share_directory(os.environ['ROBOT_DESCRIPTION_PACKAGE']), '/robots/', os.environ['ROBOT_DESCRIPTION_FILE']]
    else:
      ret['robot_description_path'] = robot_description_path
    if 'ROBOT_NS' in os.environ:
      ret['robot_ns'] = os.environ['ROBOT_NS']
    elif 'ROBOT_ID' in os.environ:
      ret['robot_ns'] = os.environ['ROBOT_ID'] + '_'
    else:
      ret['robot_ns'] = robot_ns
    if 'ROBOT_ID' in os.environ:
      ret['robot_id'] = os.environ['ROBOT_ID']
    else:
      ret['robot_id'] = robot_id

  return ret


def generate_launch_description():
  ld = launch.LaunchDescription()
  params = read_params(ld)

  robot_description_content = launch.substitutions.Command([
    launch.substitutions.PathJoinSubstitution([launch.substitutions.FindExecutable(name="xacro")]),
    ' ', params['robot_description_path'],
    ' robot_id:=', params['robot_id'],
    ' robot_ns:=', params['robot_ns'],
  ])
  # Create parameter
  robot_description_param = launch_ros.descriptions.ParameterValue(robot_description_content, value_type=str)

  ld.add_action(launch_ros.actions.PushRosNamespace(namespace=params['robot_id']))

  ld.add_action(launch_ros.actions.Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{
      'robot_description': robot_description_param,
      'publish_frequency': 100.0,
      'frame_prefix': [params['robot_id'], '/'],
    }],
  ))

  return ld

# Copyright 2022 csanrod
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

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

 
def generate_launch_description():

  project_dir = get_package_share_directory('plansys2-hospital-cavros')

  # Set the path to the Gazebo ROS package
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   

  # path to br2_tiago directory
  br2_tiago = FindPackageShare(package='br2_tiago').find('br2_tiago')
   
  # Set the path to this package.
  pkg_share = FindPackageShare(package='plansys2-hospital-cavros').find('plansys2-hospital-cavros')
 
  # Set the path to the world file
  world_file_name = 'hospital.world'
  world_path = os.path.join(pkg_share, 'worlds', world_file_name)
   
  # Set the path to the SDF model files.
  gazebo_models_path = os.path.join(pkg_share, 'models')
  os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

  # Set path to configuration file
  config_dir = os.path.join(project_dir, 'config')
  config_file = os.path.join(config_dir, 'waypoints.yaml')


 
  ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############  
  # Launch configuration variables specific to simulation
  headless = LaunchConfiguration('headless')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
 
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')
 
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')
 
  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')

  plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': pkg_share + '/pddl/project_domain.pddl' }.items())
    
  # Specify the actions
  move_cmd = Node(
    package='plansys2-hospital-cavros',
    executable='move_action_node',
    name='move_action_node',
    output='screen',
    parameters=[config_file])
    #parameters=[])

  pick_cmd = Node(
        package='plansys2-hospital-cavros',
        executable='pick_object_node',
        name='pick_object_node',
        output='screen',
        parameters=[])

  place_cmd = Node(
        package='plansys2-hospital-cavros',
        executable='place_object_node',
        name='place_object_node',
        output='screen',
        parameters=[])

  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())
 
  # Start Gazebo client
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

  # Include tiago launcher with hospital world
  tiago_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(br2_tiago, 'launch', 'sim.launch.py')))
  
  # Create the launch description and populate
  ld = LaunchDescription()
 
  # Add any actions
  #ld.add_action(tiago_sim_cmd)
  ld.add_action(plansys2_cmd)

  # Declare the launch options
  #ld.add_action(declare_simulator_cmd)
  #ld.add_action(declare_use_sim_time_cmd)
  #ld.add_action(declare_use_simulator_cmd)
  #ld.add_action(declare_world_cmd)

  ld.add_action(move_cmd)
  ld.add_action(pick_cmd)
  ld.add_action(place_cmd)

   
  return ld
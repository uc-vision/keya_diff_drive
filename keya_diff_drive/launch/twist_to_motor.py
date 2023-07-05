from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, GroupAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

  keya_diff_drive = FindPackageShare("keya_diff_drive").find('keya_diff_drive')
  default_param_file = os.path.join(keya_diff_drive, 'config', 'diff_drive.yaml')
  param_file = LaunchConfiguration('param_file')
  param_file_launch_arg = DeclareLaunchArgument(
    'param_file',
    default_value=default_param_file,
    description='Param file for the twist_to_motor node '
    )
  
  robot_state_publisher_node = Node(
    package='keya_diff_drive',
    executable='twist_to_motor_node',
    parameters=[param_file]
    )

  return LaunchDescription([
    param_file_launch_arg,
    robot_state_publisher_node
  ])

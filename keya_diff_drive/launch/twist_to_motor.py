from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
  
  keya_diff_drive_node = Node(
    package='keya_diff_drive',
    executable='twist_to_motor_node',
    parameters=[param_file]
    )

  return LaunchDescription([
    param_file_launch_arg,
    keya_diff_drive_node
  ])

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Set up directories and paths
    bringup_dir = FindPackageShare('roombav2').find('roombav2')
    config_dir = FindPackageShare('roombav2').find('roombav2') + '/config/'
    map_dir = FindPackageShare('roombav2').find('roombav2') + '/maps/'

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    params_file = LaunchConfiguration('params_file', default=os.path.join(config_dir, 'nav2_params.yaml'))

    # Include the nav2_bringup launch file
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={'map': map_dir, 'use_sim_time': use_sim_time, 'params_file': params_file}.items()
    )

    return LaunchDescription([
        bringup_cmd,
    ])

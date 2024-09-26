#nav2 bringup
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
    config_dir = os.path.join(bringup_dir, 'config')
    map_dir = os.path.join(bringup_dir, 'maps')

    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    #need to find a way to load a certain map
    map_file = LaunchConfiguration('dll_couloir', default=os.path.join(map_dir, 'dll_couloir.yaml'))

    #yaml config files
    amcl_params_file = os.path.join(config_dir, 'amcl_params.yaml')
    costmap_params_file = os.path.join(config_dir, 'costmap_params.yaml')
    ekf_params_file = os.path.join(config_dir, 'ekf.yaml')
    planner_params_file = os.path.join(config_dir, 'planner_params.yaml')
    slam_toolbox_params_file = os.path.join(config_dir, 'slam_toolbox_params.yaml')

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params_file, {'use_sim_time': use_sim_time}],
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': use_sim_time}]
    )

    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='costmap_2d',
        name='costmap_2d',
        output='screen',
        parameters=[costmap_params_file, {'use_sim_time': use_sim_time}],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
    )

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_params_file, {'use_sim_time': use_sim_time}],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_toolbox_params_file, {'use_sim_time': use_sim_time}],
    )


    return LaunchDescription([
        amcl_node,
        map_server_node,
        costmap_node,
        ekf_node,
        planner_node,
        slam_toolbox_node,
    ])

#LAUNCH FILE FOR SLAM AND MANUAL CONTROL

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare(package='roombav2').find('roombav2')
    default_model_path = os.path.join(pkg_share, 'models/microver.urdf.xacro')
    slam_config_path = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    lidar_launch_file_path = FindPackageShare('sllidar_ros2').find('sllidar_ros2') + '/launch/sllidar_c1_launch.py'
    robot_localization_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    twist_mux_config_path = os.path.join(pkg_share, 'config', 'twist_mux.yaml')

    model = LaunchConfiguration('model', default=default_model_path)
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(Command(['xacro ', model]), value_type=str)
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config_path],
    )


    joystick_control_node = Node(
        package='roombav2',
        executable='joystick_motor_controller',
        name='joystick_motor_controller',
        output='screen'
    )

    system_info_node = Node(
        package='roombav2',
        executable='system_info',
        name='system_info',
        output='screen'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_path]
    )

    robot_localization_node = Node(  #does odom -> base_link dynamic tf using ekf with odom and later imu
        package = 'robot_localization',
        executable = 'ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_config_path, {'use_sim_time': use_sim_time}],
    )

    lidar_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([lidar_launch_file_path]),
        launch_arguments={'parameter_name': 'parameter_value'}.items()
    )

    static_tf_base_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_base_to_lidar_link',
        arguments=['0', '0', '0.168', '0', '0', '3.141592653589793', 'base_link', 'lidar_link']  # x, y, z, yaw, pitch, roll
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=default_model_path,
            description='Absolute path to robot URDF file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher,
        joint_state_publisher_node,
        system_info_node,
        joystick_control_node,
        static_tf_base_lidar,
        lidar_launch_include,
        slam_toolbox_node,
        robot_localization_node,
    ])


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

    encoder_reader_node = Node(
        package='roombav2',
        executable='encoder_reader',
        name='encoder_reader',
        output='screen'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_path]
    )

    lidar_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([lidar_launch_file_path]),
        launch_arguments={'parameter_name': 'parameter_value'}.items()  # Adjust these as necessary
    )

    static_tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_odom_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']  # x, y, z, yaw, pitch, roll
    )

    static_tf_base_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_base_to_lidar_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']  # x, y, z, yaw, pitch, roll
    )

    wtf_is_this = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='scan_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'scan']  # x, y, z, yaw, pitch, roll
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
        encoder_reader_node,
        slam_toolbox_node,
        static_tf_odom_base,
        static_tf_base_lidar,
        lidar_launch_include  # Add this line to include the Lidar launch
    ])


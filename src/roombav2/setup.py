from setuptools import setup, find_packages

package_name = 'roombav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, [
            package_name+'/encoder_reader.py', 
            package_name+'/joystick_motor_controller.py', 
            package_name+'/nav2_motor_controller.py',
            package_name+'/system_info.py',
            package_name+'/pwm_open_loop.py',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/localization_launch.py',
            'launch/navigation_launch.py',
            'launch/microver.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/amcl_params.yaml', 
            'config/costmap_params.yaml', 
            'config/planner_params.yaml', 
            'config/slam_toolbox_params.yaml',
            'config/ekf.yaml',
            'config/twist_mux.yaml',
            'config/nav2_params.yaml',
        ]),
        ('share/' + package_name + '/models', [
            'models/microver.urdf.xacro', 
            'models/constants.xacro', 
            'models/inertial_macros.xacro',
        ]),
        ('share/' + package_name + '/maps', [
            'maps/DLL.pgm',
            'maps/DLL.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arno',
    maintainer_email='arno.laurie@epfl.ch',
    description='Receives PS4 joystick inputs from the CS and sends the wheel speeds to the master Arduino according to differential drive kinematics.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_reader = roombav2.encoder_reader:main',
            'nav2_motor_controller = roombav2.nav2_motor_controller:main',
            'joystick_motor_controller = roombav2.joystick_motor_controller:main',
            'system_info = roombav2.system_info:main',
            'pwm_open_loop = roombav2.pwm_open_loop:main',
        ],
    },
)

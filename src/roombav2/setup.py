from setuptools import setup

package_name = 'roombav2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/nav2_bringup.launch.py',
            'launch/microver.launch.py'
        ]),
        ('share/' + package_name + '/config', ['config/amcl_params.yaml', 'config/costmap_params.yaml', 'config/planner_params.yaml', 'config/slam_toolbox_params.yaml']),
        #('share/' + package_name + '/maps', ['maps/some_map.pgm']),
        ('share/' + package_name + '/models', ['models/microver.urdf.xacro', 'models/constants.xacro', 'models/inertial_macros.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='arno.laurie@epfl.ch',
    description='Description of the package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_reader = roombav2.encoder_reader:main',
            'nav2_motor_controller = roombav2.nav2_motor_controller:main'
        ],
    },
)

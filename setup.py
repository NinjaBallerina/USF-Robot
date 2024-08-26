from setuptools import setup

package_name = 'usf_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_joystick.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Farida',
    maintainer_email='fwinters@usf.edu',
    description='ROS2 package for controlling a 4-DOF robot with Pololu Micro Maestro 6-channel controller and 8BitDo Ultimate C joystick.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = usf_robot.robot_controller:main',
            'joystick_control = usf_robot.joystick_control:main',
        ],
    },
)

from setuptools import setup

package_name = 'ros2_mujoco_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yx-xx',
    maintainer_email='xx@xx.com',
    description='ROS2 MuJoCo机器人仿真包',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = ros2_mujoco_robot.robot_controller:main',
            'test = ros2_mujoco_robot.test:main',
        ],
    },
) 
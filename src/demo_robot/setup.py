import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'demo_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models', 'demo_robot'),
            glob('models/demo_robot/*')),
        (os.path.join('share', package_name), ['SIMULATION.md']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lydia Chheng',
    maintainer_email='chhenglydiacl@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_control = demo_robot.robot_control:main',
            'inverse_kinematic = demo_robot.inverse_kinematic:main',
            'keyboard_pad = demo_robot.keyboard_pad:main',
            'joy_pad = demo_robot.joy_pad:main',
            'sim_motor_adapter = demo_robot.sim_motor_adapter:main',
        ],
    },
)

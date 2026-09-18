import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('demo_robot')
    ros_gz_share = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(package_share, 'worlds', 'demo_world.sdf')
    model_file = os.path.join(package_share, 'models', 'demo_robot', 'model.sdf')
    bridge_file = os.path.join(package_share, 'config', 'bridge.yaml')

    control_source = LaunchConfiguration('control_source')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '--world', 'demo_world',
            '--file', model_file,
            '--name', 'demo_robot',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_file}],
        output='screen',
    )

    robot_control = Node(
        package='demo_robot',
        executable='robot_control',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    inverse_kinematic = Node(
        package='demo_robot',
        executable='inverse_kinematic',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    sim_motor_adapter = Node(
        package='demo_robot',
        executable='sim_motor_adapter',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    keyboard = Node(
        package='demo_robot',
        executable='keyboard_pad',
        condition=LaunchConfigurationEquals('control_source', 'keyboard'),
        output='screen',
        emulate_tty=True,
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
        condition=LaunchConfigurationEquals('control_source', 'gamepad'),
        output='screen',
    )
    joy_pad = Node(
        package='demo_robot',
        executable='joy_pad',
        condition=LaunchConfigurationEquals('control_source', 'gamepad'),
        output='screen',
    )
    phone_gamepad = Node(
        package='dc_controller_legacy',
        executable='gamepad_default_legacy',
        condition=LaunchConfigurationEquals('control_source', 'phone'),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'control_source',
            default_value='keyboard',
            description='Input source: keyboard, gamepad, phone, or external',
        ),
        gazebo,
        spawn_robot,
        bridge,
        robot_control,
        inverse_kinematic,
        sim_motor_adapter,
        keyboard,
        joy_node,
        joy_pad,
        phone_gamepad,
    ])

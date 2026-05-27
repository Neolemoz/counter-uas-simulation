"""Launch RT sandbox flat world + Gazebo bridge node (PLAT-RT-G6)."""

from __future__ import annotations

import os
import socket
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _gz_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        if not ip.startswith('127.') and not ip.startswith('169.254.'):
            return ip
    except OSError:
        pass
    return '127.0.0.1'


def _setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world_name').perform(context)
    session_id = LaunchConfiguration('session_id').perform(context)
    use_gui = LaunchConfiguration('use_gazebo_gui').perform(context).strip().lower() in (
        '1',
        'true',
        'yes',
        'on',
    )
    ground_snap = LaunchConfiguration('ground_snap_enabled').perform(context).strip().lower() in (
        '1',
        'true',
        'yes',
        'on',
    )
    world_file = PathJoinSubstitution(
        [FindPackageShare('rt_sandbox_gz'), 'worlds', 'rt_sandbox_flat.sdf'],
    ).perform(context)

    gz_ip = LaunchConfiguration('gz_transport_ip').perform(context).strip() or '127.0.0.1'
    if gz_ip.lower() == 'auto':
        gz_ip = _gz_ip()

    if use_gui:
        gz_cmd = ['gz', 'sim', '-r', world_file]
    else:
        gz_cmd = ['gz', 'sim', '-s', '-r', world_file]

    gz = ExecuteProcess(
        cmd=gz_cmd,
        output='screen',
        additional_env={'GZ_IP': gz_ip},
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rt_sandbox_clock_bridge',
        output='screen',
        arguments=[
            f'/world/{world_name}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[(f'/world/{world_name}/clock', '/clock')],
    )

    bridge_node = Node(
        package='rt_sandbox_gz',
        executable='rt_sandbox_gz_bridge_node',
        name='rt_sandbox_gz_bridge_node',
        output='screen',
        parameters=[
            {
                'session_id': session_id,
                'world_name': world_name,
                'ground_snap_enabled': ground_snap,
                'publish_rate_hz': 10.0,
            },
        ],
    )

    return [
        SetEnvironmentVariable('GZ_IP', gz_ip),
        LogInfo(msg=f'rt_sandbox_gz: launching flat world session={session_id} GZ_IP={gz_ip}'),
        gz,
        clock_bridge,
        bridge_node,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument('session_id', description='RT session UUID'),
            DeclareLaunchArgument('world_name', default_value='rt_sandbox_flat'),
            DeclareLaunchArgument(
                'use_gazebo_gui',
                default_value='false',
                description='Headless by default; true for maintainer visual review',
            ),
            DeclareLaunchArgument(
                'ground_snap_enabled',
                default_value='true',
                description='Apply model z offset for ground placement',
            ),
            DeclareLaunchArgument(
                'gz_transport_ip',
                default_value='127.0.0.1',
                description='GZ_IP for transport; auto for legacy discovery',
            ),
            OpaqueFunction(function=_setup),
        ],
    )

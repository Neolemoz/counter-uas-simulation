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

    kinematic_enabled = LaunchConfiguration('kinematic_plant_enabled').perform(context).strip().lower() in (
        '1',
        'true',
        'yes',
        'on',
    )
    publish_rate_hz = float(LaunchConfiguration('publish_rate_hz').perform(context))
    max_speed_mps = float(LaunchConfiguration('max_speed_mps').perform(context))
    max_accel_mps2 = float(LaunchConfiguration('max_accel_mps2').perform(context))
    max_turn_rate_rad_s = float(LaunchConfiguration('max_turn_rate_rad_s').perform(context))
    max_climb_mps = float(LaunchConfiguration('max_climb_mps').perform(context))
    drag_decel_per_mps = float(LaunchConfiguration('drag_decel_per_mps').perform(context))
    wind_x_mps = float(LaunchConfiguration('wind_x_mps').perform(context))
    wind_y_mps = float(LaunchConfiguration('wind_y_mps').perform(context))
    wind_z_mps = float(LaunchConfiguration('wind_z_mps').perform(context))

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
                'publish_rate_hz': publish_rate_hz,
                'kinematic_plant_enabled': kinematic_enabled,
                'max_speed_mps': max_speed_mps,
                'max_accel_mps2': max_accel_mps2,
                'max_turn_rate_rad_s': max_turn_rate_rad_s,
                'max_climb_mps': max_climb_mps,
                'drag_decel_per_mps': drag_decel_per_mps,
                'wind_x_mps': wind_x_mps,
                'wind_y_mps': wind_y_mps,
                'wind_z_mps': wind_z_mps,
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
            DeclareLaunchArgument('publish_rate_hz', default_value='10.0'),
            DeclareLaunchArgument(
                'kinematic_plant_enabled',
                default_value='true',
                description='Integrate pose commands with kinematic limits before set_pose',
            ),
            DeclareLaunchArgument('max_speed_mps', default_value='25.0'),
            DeclareLaunchArgument('max_accel_mps2', default_value='30.0'),
            DeclareLaunchArgument('max_turn_rate_rad_s', default_value='0.2792526803190757'),
            DeclareLaunchArgument('max_climb_mps', default_value='8.0'),
            DeclareLaunchArgument(
                'drag_decel_per_mps',
                default_value='0.12',
                description='Linear drag: decel (m/s^2) = coeff * speed (0 = off)',
            ),
            DeclareLaunchArgument('wind_x_mps', default_value='0.0'),
            DeclareLaunchArgument('wind_y_mps', default_value='0.0'),
            DeclareLaunchArgument('wind_z_mps', default_value='0.0'),
            OpaqueFunction(function=_setup),
        ],
    )

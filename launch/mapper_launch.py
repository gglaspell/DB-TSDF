"""Launch the mapper with independent configuration, visualization and replay."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.actions import EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _boolean(value):
    if value.lower() not in ('true', 'false'):
        raise ValueError(f'Expected true or false, got {value!r}')
    return value.lower() == 'true'


def _replay_exit(event, context):
    if event.returncode and not context.is_shutdown:
        raise RuntimeError(f'DB-TSDF replay failed (exit {event.returncode}); see replay error above')
    return [EmitEvent(event=Shutdown(reason='DB-TSDF replay finished'))]


def _setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = Path(get_package_share_directory('db_tsdf'))
    config = Path(arg('config_file')).expanduser() if arg('config_file') else share / 'config' / (arg('config') + '.yaml')
    if not config.is_file():
        raise RuntimeError(f'Configuration does not exist: {config}')
    document = yaml.safe_load(config.read_text())
    try:
        params = dict(document.get('db_tsdf_node', document.get('/**', {}))['ros__parameters'])
    except (KeyError, TypeError, AttributeError) as error:
        raise RuntimeError('Config must contain db_tsdf_node: ros__parameters:') from error
    bag = arg('bag_path')
    mode = arg('mode')
    if mode == 'auto':
        mode = 'replay' if bag else 'live'
    if bag and mode != 'replay':
        raise RuntimeError('bag_path requires mode:=replay or mode:=auto')
    clock = arg('use_sim_time')
    params['use_sim_time'] = (mode == 'replay') if clock == 'auto' else _boolean(clock)
    params.setdefault('cloud_reliability', 'reliable' if mode == 'replay' else 'best_effort')
    params.setdefault('cloud_queue_depth', 100 if mode == 'replay' else 10)
    for name in ('in_cloud', 'fixed_frame_id', 'sensor_frame', 'output_directory', 'initial_checkpoint'):
        if arg(name):
            params[name] = arg(name)
    if arg('num_threads'):
        params['num_threads'] = int(arg('num_threads'))
    namespace = arg('namespace')
    mapper = Node(package='db_tsdf', executable='db_tsdf_node_32' if arg('mask_bits') == '32' else 'db_tsdf_node',
                  name='db_tsdf_node', namespace=namespace, output='screen', parameters=[params])
    actions = [mapper]
    if _boolean(arg('rviz')):
        rviz_config = Path(arg('rviz_config')).expanduser() if arg('rviz_config') else share / 'config' / 'rviz' / (arg('config') + '.rviz')
        if not rviz_config.is_file():
            rviz_config = share / 'config' / 'rviz' / 'college.rviz'
        frame = params.get('fixed_frame_id') or params.get('odom_frame_id', 'odom')
        actions.append(Node(package='rviz2', executable='rviz2', namespace=namespace,
                            remappings=[('/cloud', 'cloud'), ('/map_cloud', 'map_cloud')],
                            arguments=['-d', str(rviz_config), '-f', frame],
                            parameters=[{'use_sim_time': params['use_sim_time']}], output='screen'))
    if bag:
        if not (Path(bag).expanduser() / 'metadata.yaml').is_file():
            raise RuntimeError(f'bag_path must be a rosbag2 directory containing metadata.yaml: {bag}')
        replay = Node(package='db_tsdf', executable='replay.py', name='db_tsdf_replay', output='screen',
                      arguments=['--bag', str(Path(bag).expanduser().resolve()), '--rate', arg('playback_rate'),
                                 '--namespace', namespace, '--cloud-topic', params.get('in_cloud', '/os_cloud_node/points'),
                                 '--export', arg('export_format'), '--timeout', arg('replay_timeout')])
        actions.append(replay)
        if _boolean(arg('shutdown_after_replay')):
            actions.append(RegisterEventHandler(OnProcessExit(target_action=replay,
                on_exit=_replay_exit)))
    return actions


def generate_launch_description():
    defaults = {
        'config': 'college', 'config_file': '', 'mask_bits': '16', 'mode': 'auto',
        'bag_path': '', 'use_sim_time': 'auto', 'rviz': 'true', 'rviz_config': '',
        'namespace': '', 'playback_rate': '1.0', 'output_directory': '', 'num_threads': '',
        'in_cloud': '', 'fixed_frame_id': '', 'sensor_frame': '', 'initial_checkpoint': '',
        'export_format': 'pcd', 'replay_timeout': '120', 'shutdown_after_replay': 'true',
    }
    choices = {'mask_bits': ['16', '32'], 'mode': ['auto', 'live', 'replay'],
               'use_sim_time': ['auto', 'true', 'false'], 'rviz': ['true', 'false'],
               'shutdown_after_replay': ['true', 'false'],
               'export_format': ['pcd', 'ply', 'mesh', 'csv', 'checkpoint', 'none']}
    arguments = [DeclareLaunchArgument(name, default_value=value,
                 **({'choices': choices[name]} if name in choices else {})) for name, value in defaults.items()]
    return LaunchDescription(arguments + [OpaqueFunction(function=_setup)])

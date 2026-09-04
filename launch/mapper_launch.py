from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    bag_path = LaunchConfiguration('bag_path')
    config_name = LaunchConfiguration('config')
    mask_bits = LaunchConfiguration('mask_bits')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    config_file_path = PathJoinSubstitution([
        FindPackageShare('db_tsdf'),
        'config',
        PythonExpression(["'", config_name, ".yaml'"])
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('db_tsdf'),
        'config',
        'rviz',
        PythonExpression(["'", config_name, ".rviz'"])
    ])

    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--clock',
            '--rate', '1.0'
        ],
        output='screen',
        condition=IfCondition(PythonExpression(['"', bag_path, '" != ""']))
    )

    rviz_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_config_path],
        output='screen'
    )

    db_tsdf_node = Node(
        package='db_tsdf',
        executable=PythonExpression([
            "'db_tsdf_node_32' if '", mask_bits,
            "' == '32' else 'db_tsdf_node'"
        ]),
        name='db_tsdf_node',
        output='screen',
        parameters=[
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            config_file_path
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value='',
            description='Path to bag file. If empty, node expects live data.'
        ),
        DeclareLaunchArgument(
            'config',
            default_value='college',
            description='Name prefix for .yaml and .rviz files ("college", "mai").'
        ),
        DeclareLaunchArgument(
            'mask_bits',
            default_value='16',
            choices=['16', '32'],
            description='Distance-mask width. The 32-bit backend uses twice the voxel storage.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use bag /clock time. Set false for live GNSS and TF sources.'
        ),

        rviz_node,
        db_tsdf_node,
        bag_play
    ])

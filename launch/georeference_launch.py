"""Build a WGS-84 fix -> map -> odom pipeline with robot_localization.

This launch intentionally does not start a local estimator.  The local
estimator must already publish odometry in ``odom`` (typically a LiDAR/IMU or
wheel/IMU filter).  navsat_transform_node turns GNSS fixes into
``odometry/gps`` and the global EKF fuses that measurement with local odometry
to publish the REP-105 ``map -> odom`` correction consumed by DB-TSDF.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in ('true', '1', 'yes'):
        return True
    if normalized in ('false', '0', 'no'):
        return False
    raise RuntimeError(f"expected a boolean value, got '{value}'")


def _parse_datum(value: str) -> list[float]:
    try:
        datum = [float(item.strip()) for item in value.split(',')]
    except ValueError as error:
        raise RuntimeError(
            "datum must be comma-separated latitude_deg,longitude_deg,yaw_rad"
        ) from error
    if len(datum) != 3:
        raise RuntimeError(
            "datum must be comma-separated latitude_deg,longitude_deg,yaw_rad"
        )
    if not -90.0 <= datum[0] <= 90.0 or not -180.0 <= datum[1] <= 180.0:
        raise RuntimeError("datum latitude/longitude is outside the WGS-84 range")
    return datum


def _launch_setup(context):
    datum = _parse_datum(LaunchConfiguration('datum').perform(context))
    frequency = float(LaunchConfiguration('frequency').perform(context))
    use_sim_time = _as_bool(LaunchConfiguration('use_sim_time').perform(context))
    zero_altitude = _as_bool(LaunchConfiguration('zero_altitude').perform(context))

    gps_topic = LaunchConfiguration('gps_topic').perform(context)
    imu_topic = LaunchConfiguration('imu_topic').perform(context)
    local_odom_topic = LaunchConfiguration('local_odom_topic').perform(context)
    gps_odom_topic = LaunchConfiguration('gps_odom_topic').perform(context)
    global_odom_topic = LaunchConfiguration('global_odom_topic').perform(context)
    map_frame = LaunchConfiguration('map_frame').perform(context)
    odom_frame = LaunchConfiguration('odom_frame').perform(context)
    base_link_frame = LaunchConfiguration('base_link_frame').perform(context)

    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frequency': frequency,
            # A survey/configured datum makes startup and replay deterministic.
            'wait_for_datum': True,
            # robot_localization's datum parameter is latitude, longitude,
            # and heading/yaw (not altitude). The explicit ellipsoid-height
            # datum used for a DB-TSDF export is configured separately.
            'datum': datum,
            'magnetic_declination_radians': float(
                LaunchConfiguration('magnetic_declination_radians').perform(context)),
            'yaw_offset': float(LaunchConfiguration('yaw_offset').perform(context)),
            'zero_altitude': zero_altitude,
            # UTM is deliberately not advertised as the REP-105 earth frame.
            'broadcast_utm_transform': False,
            'publish_filtered_gps': False,
        }],
        remappings=[
            ('gps/fix', gps_topic),
            ('imu/data', imu_topic),
            ('odometry/filtered', local_odom_topic),
            ('odometry/gps', gps_odom_topic),
        ],
    )

    global_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frequency': frequency,
            'sensor_timeout': 0.1,
            'two_d_mode': False,
            'publish_tf': True,
            'map_frame': map_frame,
            'odom_frame': odom_frame,
            'base_link_frame': base_link_frame,
            'world_frame': map_frame,
            # Treat local odometry as a differential measurement so GPS can
            # correct map without redefining the odom frame.
            'odom0': local_odom_topic,
            'odom0_config': [
                True, True, True, False, False, True,
                False, False, False, False, False, False,
                False, False, False,
            ],
            'odom0_differential': True,
            'odom0_queue_size': 20,
            'odom1': gps_odom_topic,
            'odom1_config': [
                True, True, True, False, False, False,
                False, False, False, False, False, False,
                False, False, False,
            ],
            'odom1_differential': False,
            'odom1_queue_size': 20,
        }],
        remappings=[('odometry/filtered', global_odom_topic)],
    )
    return [navsat, global_ekf]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'datum',
            description='Required robot_localization datum: latitude_deg,longitude_deg,yaw_rad'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_link_frame', default_value='base_link'),
        DeclareLaunchArgument('gps_topic', default_value='/gps/fix'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('local_odom_topic', default_value='/odometry/local'),
        DeclareLaunchArgument('gps_odom_topic', default_value='/odometry/gps'),
        DeclareLaunchArgument('global_odom_topic', default_value='/odometry/global'),
        DeclareLaunchArgument('frequency', default_value='30.0'),
        DeclareLaunchArgument('magnetic_declination_radians', default_value='0.0'),
        DeclareLaunchArgument('yaw_offset', default_value='0.0'),
        DeclareLaunchArgument('zero_altitude', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])

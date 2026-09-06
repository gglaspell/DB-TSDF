#!/usr/bin/env python3
"""Exercise namespaced live mapping, export failures, resume, and bag replay.

Run after installing and sourcing the package: python3 test/test_ros_smoke.py
"""
import contextlib
import hashlib
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', str(180 + os.getpid() % 30))
os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.serialization import serialize_message
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import rosbag2_py
import yaml


def cloud(stamp):
    msg = PointCloud2()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    msg.height = 1
    msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(['x', 'y', 'z'])]
    points = [(1.05, (y + .5) * .1, (z + .5) * .1) for z in range(3, 27) for y in range(3, 27)]
    msg.width = len(points)
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.data = b''.join(struct.pack('<fff', *point) for point in points)
    msg.is_dense = True
    return msg


@contextlib.contextmanager
def process(command, log):
    with log.open('w') as stream:
        child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
        try:
            yield child
        except Exception:
            print(log.read_text())
            raise
        finally:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()


def wait_for(node, predicate, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        rclpy.spin_once(node, timeout_sec=.05)
    raise RuntimeError('ROS smoke test timed out')


def call(node, name, expected=True):
    client = node.create_client(Trigger, name)
    try:
        if not client.wait_for_service(timeout_sec=20):
            raise RuntimeError(f'Service unavailable: {name}')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=20)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'Service timed out: {name}')
        response = future.result()
        if response.success != expected:
            raise RuntimeError(f'{name}: {response.message}')
        try:
            return json.loads(response.message)
        except json.JSONDecodeError:
            return response.message
    finally:
        node.destroy_client(client)


def export(node, service, success=True):
    job = call(node, service)
    def done():
        statuses = call(node, 'export_status')
        return next((s for s in statuses if s['id'] == job['job_id'] and s['state'] in ['complete', 'failed']), None)
    result = wait_for(node, done)
    if result['state'] != ('complete' if success else 'failed'):
        raise RuntimeError(str(result))
    if success and service != 'publish_map':
        metadata = json.loads((Path(result['path']) / 'run.json').read_text())
        for name, digest in metadata['outputs_sha256'].items():
            actual = hashlib.sha256((Path(result['path']) / name).read_bytes()).hexdigest()
            assert actual == digest
        return result, metadata
    return result, None


def parameters(output):
    return {'in_cloud': 'input', 'fixed_frame_id': 'map', 'use_tf': True, 'use_tf_topic': False,
            'sensor_frame': 'sensor', 'tdf_grid_res': .1, 'tdf_max_cells': 27,
            'tdfGridSizeX_low': 0., 'tdfGridSizeX_high': 3., 'tdfGridSizeY_low': 0., 'tdfGridSizeY_high': 3.,
            'tdfGridSizeZ_low': 0., 'tdfGridSizeZ_high': 3., 'kernel_size': 5, 'occ_min_hits': 1,
            'bins_az': 8, 'bins_el': 8, 'min_range': 0., 'max_range': 10., 'num_threads': 2,
            'output_directory': str(output)}


def live(root, bits):
    namespace = '/smoke_' + str(bits)
    config = root / f'{bits}.yaml'
    params = parameters(root / f'maps_{bits}')
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    executable = 'db_tsdf_node' if bits == 16 else 'db_tsdf_node_32'
    command = ['ros2', 'run', 'db_tsdf', executable, '--ros-args', '-r', '__ns:=' + namespace,
               '--params-file', str(config)]
    node = rclpy.create_node('test_client', namespace=namespace)
    try:
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        publisher = node.create_publisher(PointCloud2, 'input', qos)
        tf_publisher = node.create_publisher(TFMessage, '/tf_static',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'map', 'sensor'
        transform.transform.translation.y = 1.5
        transform.transform.translation.z = 1.5
        transform.transform.rotation.w = 1.
        tf_publisher.publish(TFMessage(transforms=[transform]))
        with process(command, root / f'node_{bits}.log'):
            assert call(node, 'get_status')['ready']
            wait_for(node, lambda: publisher.get_subscription_count() > 0 and tf_publisher.get_subscription_count() > 0)
            # Retained TF has been discovered before the first cloud.
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=.05)
            invalid = cloud(1_000_000_000)
            invalid.fields = []
            publisher.publish(invalid)
            wait_for(node, lambda: call(node, 'get_status')['invalid_clouds'] == 1)
            publisher.publish(cloud(0))
            wait_for(node, lambda: call(node, 'get_status')['tf_failures'] == 1)
            for count in range(1, 4):
                publisher.publish(cloud(count * 1_000_000_000))
                wait_for(node, lambda: call(node, 'get_status')['integrated_frames'] == count)
            _, metadata = export(node, 'save_grid_pcd')
            assert metadata['integrated_frames'] == 3 and metadata['received_frames'] == 5
            assert metadata['mask_bits'] == bits and metadata['num_threads'] == 2
            _, mesh_metadata = export(node, 'save_grid_mesh')
            assert mesh_metadata['state_hash'] == metadata['state_hash']
            saved, _ = export(node, 'save_grid_checkpoint')
            checkpoint = Path(saved['path']) / 'map.dbtsdf'
            export(node, 'publish_map')
            client = node.create_client(SetParameters, 'db_tsdf_node/set_parameters')
            assert client.wait_for_service(timeout_sec=10)
            request = SetParameters.Request(parameters=[Parameter('tdf_grid_res', value=.03).to_parameter_msg()])
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            assert future.done() and not future.result().results[0].successful
            node.destroy_client(client)
            call(node, 'reset_map')
            assert call(node, 'get_status')['allocated_blocks'] == 0
            failed, _ = export(node, 'save_grid_pcd', success=False)
            assert not Path(failed['path']).exists()
        params['initial_checkpoint'] = str(checkpoint)
        config.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
        with process(command, root / f'resumed_{bits}.log'):
            assert call(node, 'get_status')['allocated_blocks'] > 0
            _, restored = export(node, 'save_grid_pcd')
            assert restored['state_hash'] == metadata['state_hash']
    finally:
        node.destroy_node()


def replay(root):
    bag = root / 'bag'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    kwargs = dict(name='/replay_test/input', type='sensor_msgs/msg/PointCloud2', serialization_format='cdr')
    try:
        topic = rosbag2_py.TopicMetadata(id=0, **kwargs)
    except TypeError:
        topic = rosbag2_py.TopicMetadata(**kwargs)  # Humble
    writer.create_topic(topic)
    for i in range(3):
        stamp = 1_000_000_000 + i * 100_000_000
        writer.write(topic.name, serialize_message(cloud(stamp)), stamp)
    del writer
    params = parameters(root / 'replay_maps')
    params.update(use_tf=False, sensor_frame='')
    config = root / 'replay.yaml'
    config.write_text(yaml.safe_dump({'db_tsdf_node': {'ros__parameters': params}}))
    command = ['ros2', 'launch', 'db_tsdf', 'mapper_launch.py', 'config_file:=' + str(config),
               'bag_path:=' + str(bag), 'namespace:=replay_test', 'rviz:=false', 'replay_timeout:=15']
    with process(command, root / 'replay.log') as child:
        if child.wait(timeout=60):
            raise RuntimeError('Replay launch failed')
    reports = list((root / 'replay_maps').glob('*/replay.json'))
    assert len(reports) == 1
    assert json.loads(reports[0].read_text())['replay']['integrated_frames'] == 3
    # A missing input topic must propagate a nonzero exit through launch.
    with process(command + ['in_cloud:=missing'], root / 'failed_replay.log') as child:
        assert child.wait(timeout=60) != 0
    invalid = subprocess.run(['ros2', 'run', 'db_tsdf', 'db_tsdf_node', '--ros-args',
        '-p', 'tdf_grid_res:=0.0', '-p', 'output_directory:=' + str(root / 'invalid')],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=20)
    assert invalid.returncode != 0


def legacy(root):
    params = parameters(root / 'legacy_maps')
    params.update(use_tf_topic=True, sensor_frame='', in_tf_topic='pose',
                  legacy_allow_nearest=False, legacy_max_skew=.2)
    config = root / 'legacy.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    node = rclpy.create_node('legacy_test_client', namespace='/legacy_test')
    try:
        publisher = node.create_publisher(PointCloud2, 'input',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        poses = node.create_publisher(TransformStamped, 'pose', 10)
        corrected = []
        subscription = node.create_subscription(PointCloud2, 'cloud', corrected.append,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        command = ['ros2', 'run', 'db_tsdf', 'db_tsdf_node', '--ros-args',
                   '-r', '__ns:=/legacy_test', '--params-file', str(config)]
        with process(command, root / 'legacy.log'):
            call(node, 'get_status')
            wait_for(node, lambda: publisher.get_subscription_count() and poses.get_subscription_count())
            # Deliberately deliver poses in reverse timestamp order.
            for stamp, x in [(1_200_000_000, .2), (1_000_000_000, 0.)]:
                pose = TransformStamped()
                pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
                pose.header.frame_id, pose.child_frame_id = 'map', 'sensor'
                pose.transform.translation.x = x
                pose.transform.rotation.w = 1.
                poses.publish(pose)
            bad = TransformStamped()  # Zero quaternion must be rejected.
            bad.transform.rotation.w = 0.
            poses.publish(bad)
            wait_for(node, lambda: call(node, 'get_status')['invalid_transforms'] == 1)
            publisher.publish(cloud(1_100_000_000))
            wait_for(node, lambda: corrected)
            assert abs(struct.unpack_from('<f', corrected[-1].data)[0] - 1.15) < 1e-5
            assert call(node, 'get_status')['legacy_nearest_frames'] == 0
            publisher.publish(cloud(500_000_000))
            wait_for(node, lambda: call(node, 'get_status')['tf_failures'] == 1)
        node.destroy_subscription(subscription)
    finally:
        node.destroy_node()


def main():
    with tempfile.TemporaryDirectory(prefix='db-tsdf-ros-smoke-') as directory:
        root = Path(directory)
        os.environ['ROS_LOG_DIR'] = str(root / 'ros_logs')
        rclpy.init(args=[])
        try:
            live(root, 16)
            live(root, 32)
            legacy(root)
            replay(root)
            print('ROS smoke passed: live QoS, TF, validation, snapshots, checkpoints, namespaces, and replay')
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    main()

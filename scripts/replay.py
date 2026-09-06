#!/usr/bin/env python3
"""Wait for a mapper, replay every frame, verify accounting, and await export."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile
import time

import rclpy
from rclpy.utilities import remove_ros_args
from std_srvs.srv import Trigger
import yaml


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--cloud-topic', required=True)
    parser.add_argument('--namespace', default='')
    parser.add_argument('--rate', type=float, default=1)
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--export', choices=['pcd', 'ply', 'mesh', 'csv', 'checkpoint', 'none'], default='pcd')
    args = parser.parse_args(remove_ros_args(args=argv)[1:])
    if not math.isfinite(args.rate) or args.rate <= 0 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('rate and timeout must be finite and positive')
    metadata = yaml.safe_load((args.bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    rclpy.init(args=[])
    node = rclpy.create_node('db_tsdf_replay_client', namespace=args.namespace)
    player = None
    try:
        topic = node.resolve_topic_name(args.cloud_topic)
        expected = sum(t['message_count'] for t in metadata['topics_with_message_count']
                       if t['topic_metadata']['name'] == topic)
        if not expected:
            raise RuntimeError(f'Bag contains no messages on {topic}')
        clients = {}

        def call(name):
            if name not in clients:
                clients[name] = node.create_client(Trigger, name)
            client = clients[name]
            if not client.wait_for_service(timeout_sec=min(args.timeout, 30)):
                raise RuntimeError(f'Mapper service unavailable: {name}')
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=min(args.timeout, 30))
            if not future.done() or future.result() is None:
                raise RuntimeError(f'Mapper service timed out: {name}')
            result = future.result()
            if not result.success:
                raise RuntimeError(f'{name} failed: {result.message}')
            return json.loads(result.message)

        before = call('get_status')
        expected += before['received_frames']
        with tempfile.TemporaryDirectory(prefix='db-tsdf-replay-') as directory:
            qos = Path(directory) / 'qos.yaml'
            qos.write_text(yaml.safe_dump({topic: {'reliability': 'reliable', 'history': 'keep_last', 'depth': 1000}}))
            player = subprocess.Popen(['ros2', 'bag', 'play', str(args.bag), '--clock', '--delay', '1',
                                       '--rate', str(args.rate), '--qos-profile-overrides-path', str(qos)])
            while player.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.2)
            if player.returncode:
                raise RuntimeError(f'ros2 bag play failed with exit code {player.returncode}')
        deadline = time.monotonic() + args.timeout
        status = call('get_status')
        while status['received_frames'] < expected and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            status = call('get_status')
        received = status['received_frames'] - before['received_frames']
        integrated = status['integrated_frames'] - before['integrated_frames']
        bag_frames = expected - before['received_frames']
        if received != bag_frames or integrated != bag_frames:
            raise RuntimeError(f'Incomplete replay: expected {bag_frames}, received {received}, integrated {integrated}. Status: {json.dumps(status)}')
        if status['skipped_blocks'] > before['skipped_blocks'] or status['evictions'] > before['evictions']:
            raise RuntimeError('Replay exceeded map capacity; increase tdf_max_cells before comparing map quality')
        if args.export != 'none':
            job = call('save_grid_' + args.export)
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline:
                jobs = call('export_status')
                result = next((item for item in jobs if item['id'] == job['job_id']), None)
                if result and result['state'] == 'complete':
                    summary = {'replay': status, 'export': result, 'bag': str(args.bag.resolve()),
                               'expected_frames': bag_frames, 'playback_rate': args.rate,
                               'input_sha256': {}}
                    for name in ['metadata.yaml', *metadata['relative_file_paths']]:
                        path = (args.bag / name).resolve()
                        if not path.is_relative_to(args.bag.resolve()):
                            raise RuntimeError('Bag metadata refers outside the bag directory')
                        digest = hashlib.sha256()
                        with path.open('rb') as stream:
                            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                                digest.update(chunk)
                        summary['input_sha256'][name] = digest.hexdigest()
                    # Hashes describe files present at completion. The offline
                    # runner also checks that input files did not change in use.
                    output = Path(result['path']) / 'replay.json'
                    temporary = output.with_suffix('.json.partial')
                    temporary.write_text(json.dumps(summary, indent=2) + '\n')
                    temporary.replace(output)
                    print(json.dumps(summary, indent=2), flush=True)
                    return 0
                if result and result['state'] == 'failed':
                    raise RuntimeError(result['error'])
                rclpy.spin_once(node, timeout_sec=0.2)
            raise RuntimeError('Export did not finish before timeout')
        print(json.dumps(status, indent=2), flush=True)
        return 0
    finally:
        if player is not None and player.poll() is None:
            player.terminate()
            try:
                player.wait(timeout=10)
            except subprocess.TimeoutExpired:
                player.kill()
                player.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, OSError, KeyError) as error:
        raise SystemExit(f'DB-TSDF replay failed: {error}')

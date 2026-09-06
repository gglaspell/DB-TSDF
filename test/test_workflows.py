#!/usr/bin/env python3
"""Artifact integrity, repeatability, and safe dataset-cache failure paths."""
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('download_dataset', ROOT / 'scripts/download_dataset.py')
dataset = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dataset)
RUNNER = sys.argv.pop(1) if len(sys.argv) > 1 else None


class Workflows(unittest.TestCase):
    def test_offline(self):
        if not RUNNER:
            self.skipTest('pass the offline executable as the first argument')
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for bits in [16, 32]:
                hashes = set()
                for workers in [1, 4]:
                    output = root / f'{bits}_{workers}'
                    subprocess.run([RUNNER, '--synthetic', '--output', str(output), '--threads', str(workers),
                                    '--mask-bits', str(bits)], check=True, stdout=subprocess.DEVNULL)
                    run = json.loads((output / 'run.json').read_text())
                    self.assertEqual(run['expected_frames'], run['integrated_frames'])
                    self.assertEqual(run['dropped_frames'], 0)
                    self.assertEqual(run['evictions'], 0)
                    self.assertGreater(run['quality']['recall'], .99)
                    self.assertLess(run['quality']['accuracy_m']['max'], .101)
                    for name, expected in run['outputs_sha256'].items():
                        self.assertEqual(dataset.sha256(output / name), expected)
                    hashes.add(run['state_hash'])
                    # The generated inputs can be replayed as an external manifest.
                    replay = root / (output.name + '_replay')
                    subprocess.run([RUNNER, '--manifest', str(output / 'inputs/manifest.json'),
                                    '--output', str(replay), '--mask-bits', str(bits)],
                                   check=True, stdout=subprocess.DEVNULL)
                    self.assertEqual(json.loads((replay / 'run.json').read_text())['state_hash'], run['state_hash'])
                self.assertEqual(len(hashes), 1)
            existing = root / 'existing'
            existing.mkdir()
            sentinel = existing / 'sentinel'
            sentinel.write_text('keep')
            failed = subprocess.run([RUNNER, '--synthetic', '--output', str(existing)], capture_output=True)
            self.assertNotEqual(failed.returncode, 0)
            self.assertEqual(sentinel.read_text(), 'keep')

    def test_cache_integrity(self):
        with tempfile.TemporaryDirectory() as temporary:
            cache = Path(temporary)
            archive = cache / 'mai_city.tar.gz'
            archive.write_bytes(b'first download')
            _, digest = dataset.cached_archive(cache)
            self.assertEqual(digest, dataset.sha256(archive))
            archive.write_bytes(b'changed download')
            with self.assertRaisesRegex(RuntimeError, 'checksum mismatch'):
                dataset.cached_archive(cache)

    def test_conversion_preserves_outputs(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            archive_path = root / 'fixture.tar.gz'
            with tarfile.open(archive_path, 'w:gz') as archive:
                member = tarfile.TarInfo('../../bags/01.bag')
                member.size = 4
                archive.addfile(member, io.BytesIO(b'data'))
            with tarfile.open(archive_path, 'r:gz') as archive:
                member = dataset.select_members(archive, ['01'])['01']
                def converter(command, **unused):
                    destination = Path(command[-1])
                    destination.mkdir()
                    (destination / 'metadata.yaml').write_text('fixture')
                    (destination / 'data.db3').write_bytes(b'converted')
                with patch.object(dataset.subprocess, 'run', side_effect=converter) as run:
                    dataset.convert_sequence(archive, member, '01', root, 'abc', Path('converter'), {'version': 1})
                    dataset.convert_sequence(archive, member, '01', root, 'abc', Path('converter'), {'version': 1})
                    self.assertEqual(run.call_count, 1)
                (root / '01/data.db3').write_bytes(b'user data')
                with self.assertRaisesRegex(RuntimeError, 'preserved'):
                    dataset.convert_sequence(archive, member, '01', root, 'abc', Path('converter'), {'version': 1})
                self.assertEqual((root / '01/data.db3').read_bytes(), b'user data')
                with patch.object(dataset.subprocess, 'run', side_effect=subprocess.CalledProcessError(1, 'convert')):
                    with self.assertRaises(subprocess.CalledProcessError):
                        dataset.convert_sequence(archive, member, '02', root, 'abc', Path('converter'), {'version': 1})
                self.assertFalse((root / '02').exists())
                self.assertFalse(list(root.glob('.sequence-*')))


if __name__ == '__main__':
    unittest.main()

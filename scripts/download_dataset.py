#!/usr/bin/env python3
"""Cache the complete MaiCity archive and atomically convert selected sequences."""
import argparse
import fcntl
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile

URL = 'https://www.ipb.uni-bonn.de/html/projects/mai_city/mai_city.tar.gz'
ROSBAGS_VERSION = '0.10.11'


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def hashes(directory):
    return {p.relative_to(directory).as_posix(): sha256(p)
            for p in sorted(directory.rglob('*')) if p.is_file() and p.name != 'conversion.json'}


def write_json(path, data):
    temporary = path.with_name(path.name + '.partial')
    temporary.write_text(json.dumps(data, indent=2) + '\n')
    temporary.replace(path)


def cached_archive(cache, expected=None):
    archive = cache / 'mai_city.tar.gz'
    receipt = cache / 'archive.json'
    if not archive.is_file():
        partial = archive.with_suffix('.gz.partial')
        print('Downloading the FULL MaiCity archive (several GB), even for one sequence.', flush=True)
        subprocess.run(['wget', '--continue', '--output-document', str(partial), URL], check=True)
        digest = sha256(partial)
        if expected and digest != expected:
            raise RuntimeError('Archive SHA-256 mismatch; cached partial retained for inspection')
        partial.replace(archive)
    digest = sha256(archive)
    previous = json.loads(receipt.read_text()) if receipt.exists() else None
    if (expected and digest != expected) or (previous and digest != previous['sha256']):
        raise RuntimeError('Cached archive checksum mismatch; inspect cache before retrying')
    write_json(receipt, {'url': URL, 'sha256': digest, 'bytes': archive.stat().st_size,
                        'trusted_reference_supplied': bool(expected) or bool(previous and previous.get('trusted_reference_supplied'))})
    return archive, digest


def converter_environment(cache):
    environment = cache / ('rosbags-' + ROSBAGS_VERSION)
    python = environment / 'bin/python'
    marker = environment / 'ready.json'
    if marker.exists():
        info = json.loads(marker.read_text())
        installed = subprocess.check_output([str(python), '-m', 'pip', 'freeze'], text=True)
        if installed != info['pip_freeze']:
            raise RuntimeError('Cached converter dependencies changed; use a new cache directory')
        return environment / 'bin/rosbags-convert', info
    if not (environment / 'bin/pip').exists():
        try:
            subprocess.run([sys.executable, '-m', 'venv', str(environment)], check=True)
        except subprocess.CalledProcessError as error:
            raise RuntimeError('Creating converter environment failed; install python3-venv and retry') from error
    subprocess.run([str(python), '-m', 'pip', 'install', 'rosbags==' + ROSBAGS_VERSION], check=True)
    info = {'rosbags': ROSBAGS_VERSION,
            'python': subprocess.check_output([str(python), '--version'], text=True).strip(),
            'pip_freeze': subprocess.check_output([str(python), '-m', 'pip', 'freeze'], text=True)}
    write_json(marker, info)
    return environment / 'bin/rosbags-convert', info


def select_members(archive, sequences):
    selected = {}
    for member in archive:
        path = Path(member.name)
        if not member.isfile() or path.suffix != '.bag' or 'bags' not in path.parts:
            continue
        numbers = re.findall(r'\d+', path.stem)
        if not numbers:
            continue
        sequence = numbers[-1].zfill(2)
        if sequence in sequences:
            if sequence in selected:
                raise RuntimeError(f'Ambiguous archive: multiple bags for sequence {sequence}')
            selected[sequence] = member
    missing = set(sequences) - selected.keys()
    if missing:
        raise RuntimeError(f'Archive missing sequences: {sorted(missing)}')
    return selected


def convert_sequence(archive, member, sequence, output, digest, converter, environment):
    destination = output / sequence
    receipt = destination / 'conversion.json'
    if destination.exists():
        if not receipt.is_file():
            raise RuntimeError(f'Existing output has no conversion receipt; preserved: {destination}')
        previous = json.loads(receipt.read_text())
        if (previous.get('archive_sha256') != digest or previous.get('converter') != environment or
                not (destination / 'metadata.yaml').is_file() or previous.get('outputs_sha256') != hashes(destination)):
            raise RuntimeError(f'Existing output failed verification; preserved: {destination}')
        print(f'Verified existing sequence {sequence}: {destination}', flush=True)
        return
    # Read the selected regular member into a controlled filename. Archive
    # paths and symlinks are never extracted into the filesystem.
    with tempfile.TemporaryDirectory(prefix=f'.sequence-{sequence}-', dir=output) as temporary:
        staging = Path(temporary)
        bag = staging / 'source.bag'
        with archive.extractfile(member) as source, bag.open('wb') as target:
            shutil.copyfileobj(source, target, length=1024 * 1024)
        converted = staging / 'converted'
        subprocess.run([str(converter), '--src', str(bag), '--dst-version', '8', '--dst', str(converted)], check=True)
        if not (converted / 'metadata.yaml').is_file():
            raise RuntimeError('Converter did not produce rosbag2 metadata')
        write_json(converted / 'conversion.json', {
            'schema': 'db-tsdf-conversion/v1', 'sequence': sequence, 'source_url': URL,
            'archive_sha256': digest, 'archive_member': member.name, 'ros1_bag_sha256': sha256(bag),
            'converter': environment, 'outputs_sha256': hashes(converted)})
        converted.rename(destination)
    print(f'Ready: {destination}', flush=True)


def main():
    repo = Path(__file__).resolve().parents[1]
    workspace = repo.parent.parent if repo.parent.name == 'src' else repo
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sequences', nargs='+', choices=['00', '01', '02'], default=['01'])
    parser.add_argument('--output', type=Path, default=workspace / 'datasets/mai_city')
    parser.add_argument('--cache', type=Path, default=workspace / 'datasets/.cache/mai_city')
    parser.add_argument('--sha256', help='Trusted archive digest; otherwise check against the first downloaded copy')
    args = parser.parse_args()
    if args.sha256 and not re.fullmatch('[0-9a-fA-F]{64}', args.sha256):
        parser.error('--sha256 must contain 64 hexadecimal characters')
    args.output = args.output.expanduser().resolve()
    args.cache = args.cache.expanduser().resolve()
    args.cache.mkdir(parents=True, exist_ok=True)
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / '.convert.lock').open('a') as output_lock, (args.cache / '.download.lock').open('a') as cache_lock:
        fcntl.flock(output_lock, fcntl.LOCK_EX)
        fcntl.flock(cache_lock, fcntl.LOCK_EX)
        converter, environment = converter_environment(args.cache)
        archive_path, digest = cached_archive(args.cache, args.sha256.lower() if args.sha256 else None)
        with tarfile.open(archive_path, 'r:gz') as archive:
            members = select_members(archive, args.sequences)
            for sequence in args.sequences:
                convert_sequence(archive, members[sequence], sequence, args.output, digest, converter, environment)


if __name__ == '__main__':
    try:
        main()
    except (OSError, RuntimeError, subprocess.CalledProcessError, tarfile.TarError, ValueError) as error:
        raise SystemExit(f'MaiCity preparation failed: {error}')

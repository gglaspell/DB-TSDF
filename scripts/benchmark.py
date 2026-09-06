#!/usr/bin/env python3
"""Repeat identical offline inputs across backends/workers; fail on state drift."""
import argparse
import json
from pathlib import Path
import shutil
import statistics
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runner', default=str(Path(__file__).with_name('db_tsdf_offline')))
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--threads', nargs='+', type=int, default=[1, 2, 4])
    parser.add_argument('--mask-bits', nargs='+', type=int, choices=[16, 32], default=[16, 32])
    parser.add_argument('--repeats', type=int, default=3)
    args = parser.parse_args()
    if args.repeats < 2 or any(t < 1 or t > 1024 for t in args.threads):
        parser.error('use at least two repeats and worker counts in 1..1024')
    runner = shutil.which(args.runner)
    if not runner:
        parser.error('runner not found; pass --runner /path/to/db_tsdf_offline')
    args.output.mkdir(parents=True, exist_ok=False)
    groups, hashes = [], {}
    for bits in args.mask_bits:
        for workers in args.threads:
            runs = []
            for repeat in range(args.repeats):
                output = args.output / f'bits{bits}_threads{workers}_run{repeat}'
                command = [runner, '--output', str(output), '--threads', str(workers), '--mask-bits', str(bits)]
                command += ['--manifest', str(args.manifest.resolve())] if args.manifest else ['--synthetic']
                with (args.output / (output.name + '.log')).open('w') as log:
                    subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
                run = json.loads((output / 'run.json').read_text())
                hashes.setdefault(bits, set()).add(run['state_hash'])
                if len(hashes[bits]) != 1:
                    raise RuntimeError(f'Voxel state changed across repeats/workers for {bits}-bit backend')
                runs.append(run)
            latencies = [r['integration_ms']['p50'] for r in runs]
            groups.append({'mask_bits': bits, 'threads': workers, 'repeats': args.repeats,
                           'state_hash': runs[0]['state_hash'], 'quality': runs[0]['quality'],
                           'integration_p50_ms_mean': statistics.mean(latencies),
                           'integration_p50_ms_stdev': statistics.stdev(latencies),
                           'integration_p95_ms_mean': statistics.mean(r['integration_ms']['p95'] for r in runs),
                           'integration_p99_ms_mean': statistics.mean(r['integration_ms']['p99'] for r in runs),
                           'peak_rss_bytes_max': max(r['peak_rss_bytes'] for r in runs)})
            print(f'{bits} bits, {workers} workers: p50 {statistics.mean(latencies):.3f} ms; repeatable state', flush=True)
    result = {'schema': 'db-tsdf-benchmark/v1', 'repeatable': True,
              'input': str(args.manifest) if args.manifest else 'noise-free-plane-v1', 'groups': groups}
    (args.output / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()

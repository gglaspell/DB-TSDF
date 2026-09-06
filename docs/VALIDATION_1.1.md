# Local validation — 2026-09-05

Tested the uncommitted improvement revision based on Git commit
`a5eeedb9c72d16a16dbb1cd769d282e853754192`. The tested source fingerprint is
`2046e9ab3f824c8b4b1c12c8067444d809d262b1179d81d1bbe555bc291b2026`.
The [machine-readable baseline](../benchmarks/plane_2026-09-05.json) records
the compiler, flags, hardware, inputs, parameters and measurements.

Environment: x86_64 WSL2, Intel i9-13900K / 32 logical CPUs, GCC 15.2,
Eigen 3.4.0, PCL 1.15.1, VTK 9.5.2, ROS 2 Lyrical. Portable ROS binaries used
RelWithDebInfo (`-O2`); native core binaries used Release. Tests do not use
disabled Release assertions for their checks.

| Check | Result |
| --- | --- |
| Portable build, both backend widths | Builds without source warnings; all 5 CTest groups pass. |
| Native CPU optimization | Builds without source warnings; all 5 groups pass. |
| AddressSanitizer + UndefinedBehaviorSanitizer, Debug | All 5 groups pass; no address or UB errors. Leak detection disabled for the system-library test environment. |
| Serial reference vs destination-block workers | Identical observed voxel state, with overlapping kernels and duplicate returns. |
| Repeatability | 40 fresh-process fixture runs: 16/32 bits × 1/2/4/16 workers × 5 repeats; one state hash per backend. PCD, STL and checkpoint hashes also match within each backend. |
| Grid indexing | 3 cm nonreciprocal resolution, negative coordinates, clipped kernels, upper-bound reads, capacity, snapshot independence and setup failure preservation pass. |
| Geometry | Analytic metric plane, negative-coordinate interpolation, L2 rank inversion, unsaturated cross-backend agreement and block-seam triangle counts pass. |
| Exports | Empty-map errors, bounded job queue, failed worker status, snapshot isolation, output SHA-256 verification, corrupted/incompatible checkpoint rejection, rolling eviction-order recovery pass. |
| ROS workflow | Both executables pass namespaced live best-effort input, sensor-origin TF, malformed/zero-stamp rejection, read-only parameters, PCD/mesh/checkpoint export, map preview, reset and checkpoint restart. |
| ROS replay | Generated three-frame rosbag2 replays, drains, saves and exits successfully; missing input topic propagates a failing exit through launch. |
| Legacy pose topics | Reverse-ordered timestamps interpolate to the expected position; invalid quaternions and unbracketed timestamps are rejected. |
| Dataset tooling | Cache-tamper and output-preservation tests pass. A real tiny ROS1 bag converts with isolated `rosbags==0.10.11`, and the converted result is verified and reused. |

The original reproduced race sometimes lost occupancy hits and produced
different maps across identical runs. The new serial-reference and worker
tests agree. The original 3 cm alias and out-of-bounds read are covered by
the new integer-index and sanitizer tests.

## Synthetic timing baseline

One noiseless plane, 576 returns per frame, 20 frames per run, 10 cm voxels,
5³ L2 kernels, 16×8 directional bins and a three-hit threshold. Values below
are means of the per-run latency percentiles across five runs, in ms.

| Bits | Worker limit | p50 | p95 | p99 |
| --- | ---: | ---: | ---: | ---: |
| 16 | 1 | 0.1456 | 0.1866 | 0.2345 |
| 16 | 2 | 0.0803 | 0.1386 | 0.2429 |
| 16 | 4 | 0.0614 | 0.1101 | 0.2473 |
| 16 | 16 | 0.0484 | 0.1499 | 0.4957 |
| 32 | 1 | 0.1514 | 0.1885 | 0.2374 |
| 32 | 2 | 0.0796 | 0.1255 | 0.2527 |
| 32 | 4 | 0.0628 | 0.1418 | 0.3103 |
| 32 | 16 | 0.0647 | 0.2083 | 0.7610 |

Integration timing includes preparation, allocation/grouping and updates.
The worker limit is also bounded by the number of touched blocks. Higher
limits do not consistently improve tail latency on this small fixture.
No before/after throughput comparison with the old implementation was made.
These timings are sensitive to host scheduling and should not set production
worker counts without representative scans.

Surface voxel-center error to the reference plane: mean 0.0663 m, RMSE
0.0814 m, maximum approximately 0.1000 m. Completeness error is approximately
zero; F-score is 1.0 at the explicitly chosen 0.15 m tolerance. Surface
exports include occupied rank-0 and rank-1 voxels, so they include a band
around the plane. This point-set metric does not measure STL accuracy.

## Re-run

```bash
cmake -S . -B build -DDB_TSDF_BUILD_ROS=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j2
ctest --test-dir build --output-on-failure

cmake -S . -B build-asan -DDB_TSDF_BUILD_ROS=OFF \
  -DDB_TSDF_SANITIZERS=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build-asan -j2
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 \
  ctest --test-dir build-asan --output-on-failure

python3 scripts/benchmark.py --runner ./build/db_tsdf_offline \
  --output benchmark-results --threads 1 2 4 16 --repeats 5

# After colcon build and sourcing the ROS workspace:
python3 test/test_ros_smoke.py
```

The ROS smoke test uses a separate localhost ROS domain and temporary files.
It needs local DDS sockets. The Dockerfile and Humble/Jazzy CI jobs are
configured but were not executed in this local validation. No full MaiCity
or Newer College download/reconstruction, real-world accuracy comparison,
GNSS/ATAK client import or large-map export stress test was performed.
Architectural work such as submaps, reintegration and dynamic-object handling
is outside this revision.

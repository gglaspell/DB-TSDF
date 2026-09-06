# Reproducible runs

Use the ROS-independent runner for frame-exact comparisons. It consumes PCD
clouds and fixed poses sequentially, rejects invalid inputs or exhausted map
capacity, and publishes an output directory only after all frames and exports
succeed. `--threads` controls integration workers (default 1); `--mask-bits`
selects 16 or 32. Input order is preserved.

```bash
./build/db_tsdf_offline --synthetic --output /tmp/plane-run
./build/db_tsdf_offline --manifest /tmp/plane-run/inputs/manifest.json \
  --threads 4 --mask-bits 32 --output /tmp/plane-run-32
python3 scripts/benchmark.py --runner ./build/db_tsdf_offline \
  --manifest /path/to/inputs.json --output benchmark-results \
  --threads 1 2 4 16 --repeats 5
```

Omit `--manifest` from the benchmark for the small generated plane. Each run
uses a fresh process and output directory, so peak RSS and startup costs can
be interpreted independently. Existing output directories are rejected.

## External input manifest

Paths are relative to the manifest directory, or absolute. Each PCD needs
float32 XYZ fields. Poses and `sensor_origin_m` must be finite; the latter is
always in the fixed frame, even when the PCD is in sensor coordinates.
`cloud_to_fixed` is `[tx, ty, tz, qx, qy, qz, qw]` with a unit quaternion; it
defaults to identity. Positive integer timestamps must increase strictly.
Deskew clouds before creating the manifest when appropriate.

```json
{
  "schema": "db-tsdf-input/v1",
  "fixed_frame": "map",
  "parameters": {
    "tdf_grid_res": 0.05,
    "tdf_max_cells": 1000,
    "tdfGridSizeX_low": -10.0,
    "tdfGridSizeX_high": 10.0,
    "tdfGridSizeY_low": -10.0,
    "tdfGridSizeY_high": 10.0,
    "tdfGridSizeZ_low": -3.0,
    "tdfGridSizeZ_high": 5.0,
    "kernel_size": 5,
    "occ_min_hits": 1,
    "bins_az": 60,
    "bins_el": 60,
    "shadow_radius": 3,
    "distance_mode": "L2",
    "mesh_mode": "distance"
  },
  "frames": [
    {
      "pcd": "clouds/000001.pcd",
      "stamp_ns": 1000000000,
      "cloud_to_fixed": [1, 0, 0, 0, 0, 0, 1],
      "sensor_origin_m": [1, 0, 0]
    }
  ],
  "reference_pcd": "reference.pcd",
  "evaluation_tolerance_m": 0.10
}
```

The reference PCD and tolerance are optional. Supported parameters are the
grid bounds/resolution/capacity, kernel settings, `capacity_policy`,
`memory_budget_mb`, `mesh_mode`, `mesh_smoothing_sigma`, `pc_downsampling`,
`min_range`, and `max_range`. Unknown parameter names fail so typos cannot
silently change an experiment. Effective defaults are expanded in `run.json`.
Keep calibration and pose conversion instructions with the input manifest;
its checksum covers every inline pose and sensor origin.

## Interpreting results

`run.json` records source fingerprint and available Git revision/dirty state,
compiler/build flags, Eigen/PCL/VTK versions, CPU/OS, worker count, complete
parameters, input checksums, frame/point accounting, capacity, canonical voxel
state, process peak RSS, and output checksums. External inputs are hashed on
first use and checked again after the run. Concurrent input changes fail.

Integration timing includes point/bin preparation, block allocation/grouping
and voxel updates. Frame total additionally includes cloud loading and
transformation/filtering; the first use includes input hashing. Both provide
mean, p50, p95, p99 and maximum in milliseconds. Setup, export and total wall
times are separate. Peak RSS includes libraries, input and export buffers,
not just voxels. Total wall time ends after output hashing, before writing
the report and renaming the completed directory.

Accuracy measures occupied surface voxel centers against the nearest
reference point; completeness measures the reverse direction. Distances
include mean, RMSE and percentiles. Precision and recall are the fractions
within `evaluation_tolerance_m`; F-score is their harmonic mean. These are
point-set metrics, not mesh-to-surface distances. Reference sampling density
affects them. The generated fixture is one noiseless plane, not an indoor or
outdoor mapping benchmark. Evaluate thin objects, edges, sensor noise and
real sequences before promoting mesh modes or presets.

The benchmark checks voxel hashes across repeats and worker counts within
each backend and writes `summary.json` with latency variation. Hash bytes
differ between 16-bit and 32-bit backends even when unsaturated ranks agree;
the integration regression test compares their voxel fields directly.

## ROS replay

Launch with `bag_path` to wait for mapper readiness, replay with `/clock`,
check received and integrated cloud counts, reject capacity loss, and wait
for the selected export. Use an isolated topic/namespace/domain without a
concurrent live publisher. Reduce `playback_rate` or increase a bounded queue
when incomplete input is reported. Failure propagates through launch when
the default `shutdown_after_replay:=true` is used.

Completed exports include a `replay.json` with playback settings, expected
frame count and bag-file SHA-256 values. These are hashes of the bag files at
completion; the offline runner additionally checks for changes during use.
With `export_format:=none`, accounting is printed without a saved sidecar.
TF availability can still depend on message timing, so count-checked ROS
playback and deterministic offline evaluation serve different purposes.

## Build and data provenance

The source fingerprint hashes mapper/core headers, C++ sources, Python
scripts, launch files, YAML configuration, CMake input and package metadata. It captures
uncommitted source changes. Git information may be unavailable in a Docker
context without `.git`; the content fingerprint is still available.

The Docker base is digest pinned and the image records `system-packages.txt`.
Apt packages are resolved during the build, so this is not a bit-for-bit
reproducible toolchain. Keep the built image digest and package list alongside
benchmark artifacts for strict comparisons. CI is configured for Humble and
Jazzy plus a sanitizer core build. Local test evidence is recorded separately
in [VALIDATION_1.1.md](VALIDATION_1.1.md).

Dataset conversion uses `rosbags==0.10.11`, rosbag2 format version 8, and a
private virtual environment. Dependency versions are frozen into the receipt
and checked on reuse; newly created environments can resolve different
transitive versions. Preserve the environment/receipt for strict repeat runs.
An archive digest supplied with `--sha256` is checked before conversion.
Without that, the first-download hash is a local integrity baseline, not an
independently verified publisher checksum. The full archive remains in the
cache so downloads can resume. Existing conversions are never replaced on
verification or conversion failure.

The converter's installed `--help` is authoritative for the pinned release:
0.10.11 uses `--src` and `--dst`. Current online
[Rosbags conversion documentation](https://ternaris.gitlab.io/rosbags/topics/convert.html)
may describe another release. MaiCity's official
[dataset page](https://www.ipb.uni-bonn.de/data/mai-city-dataset/) describes the
available sequences and sensor/pose data.

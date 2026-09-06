<h1 align="center">DB-TSDF: Directional Bitmask-based Truncated Signed Distance Fields for Efficient Volumetric Mapping</h1>

<h3 align="center">
  Accepted at the <a href="https://2026.ieee-icra.org/">IEEE International Conference on Robotics and Automation (ICRA) 2026</a>
</h3>

<p align="center">
  <a href="https://robotics-upo.github.io/DB-TSDF/"><img src="https://img.shields.io/badge/Project-Website-007acc?style=flat" alt="Project Website"></a>
  <a href="https://youtu.be/yqLLAqjWyFM"><img src="https://img.shields.io/badge/YouTube-Video-red?style=flat&logo=youtube" alt="YouTube Video"></a>
  <a href="https://arxiv.org/abs/2509.20081"><img src="https://img.shields.io/badge/arXiv-Paper-b31b1b?style=flat&logo=arxiv" alt="arXiv Paper"></a>
</p>


**DB-TSDF** presents a high-efficiency, CPU-only framework for volumetric mapping. It utilizes a novel directional bitmask-based integration scheme to incrementally fuse LiDAR data into a dense voxel grid.

Key features include:
- **Directional Kernels:** Efficiently model beam geometry and occlusion in 3D.
- **Bitmask Encoding:** Constant-cost mask operations for each voxel update; total scan work depends on points, kernel size and allocation.
- **High Performance:** Multi-threaded C++ implementation fully integrated with ROS 2.

The design prioritizes predictable runtime and high-resolution reconstruction, making it an ideal solution for robotic platforms with limited GPU resources. 

![Example reconstruction](docs/media/college_tittle.png)




## Build and first run

Use ROS 2 Humble on Ubuntu 22.04 or Jazzy on Ubuntu 24.04. CI is configured
for both combinations; the current changes were also tested locally on ROS 2
Lyrical. The core and offline runner can build without ROS. Portable binaries
are the default; `-DDB_TSDF_NATIVE=ON` opts into CPU-specific instructions.

From a sourced ROS workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/robotics-upo/DB-TSDF.git
cd ~/ros2_ws
rosdep install --from-paths src/DB-TSDF --ignore-src -r -y
colcon build --packages-select db_tsdf --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run db_tsdf db_tsdf_offline --synthetic --output /tmp/db-tsdf-demo
```

The last command needs no dataset download or ROS topics. It creates a small
plane fixture, `map.pcd`, `mesh.stl`, a resumable `map.dbtsdf` checkpoint, and
`run.json` with checksums and measurements. Choose a new output directory for
each run; existing outputs are preserved. This fixture checks the software,
and does not establish dataset accuracy or a speedup.

For a core-only build, install Eigen3, PCL, VTK9, OpenMP, OpenSSL and
nlohmann-json development packages, then run:

```bash
cmake -S . -B build -DDB_TSDF_BUILD_ROS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
./build/db_tsdf_offline --synthetic --output /tmp/db-tsdf-demo
```

The Dockerfile builds **the supplied checkout** and uses a base-image digest.
It records installed system packages; later apt packages are not snapshot
pinned. From the repository root:

```bash
docker build -t db-tsdf --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) .
mkdir -p maps
docker run --rm -v "$PWD/maps:/outputs" db-tsdf \
  ros2 run db_tsdf db_tsdf_offline --synthetic --output /outputs/demo
```

## Live mapping and bag replay

```bash
# Live mode uses wall time and accepts best-effort sensor publishers.
ros2 launch db_tsdf mapper_launch.py config:=mai mode:=live rviz:=false

# Start the mapper, wait for readiness, play with /clock, drain input, save,
# verify completion, and shut down. Incomplete replay exits with an error.
ros2 launch db_tsdf mapper_launch.py config:=mai rviz:=false \
  bag_path:=/absolute/path/to/mai_city/01 output_directory:=/tmp/mai-maps
```

Set `mask_bits:=32` to select the 32-bit executable. It uses 8-byte voxels,
compared with 4 bytes for the default 16-bit backend. More bits only help when
the configured kernel needs more distance ranks.

Launch arguments include `config_file`, `rviz`, `rviz_config`, `namespace`,
`playback_rate`, `num_threads`, `in_cloud`, `fixed_frame_id`, `sensor_frame`,
`output_directory`, `initial_checkpoint`, and `export_format`. YAML and RViz
selection are independent. `mode:=auto` selects replay when `bag_path` is
provided; `use_sim_time:=auto` follows the mode. Set `shutdown_after_replay:=false`
for interactive inspection after playback. Manual replay requires
`mode:=replay` and `ros2 bag play BAG --clock` in another terminal.

Replay verifies cloud counts and export completion. Live ROS queues are
bounded and can drop data under load; the replay command detects incomplete
runs instead of accepting them as evaluation results. Use the offline runner
with fixed clouds and poses for deterministic comparisons.

Inputs must have float32 XYZ fields, finite coordinates, and timestamps.
Supply deskewed clouds when sensor motion matters. `fixed_frame_id` is the
integration frame. When points are already in that frame, set `sensor_frame`
to the physical sensor's TF frame so ray direction and range filtering use
the correct origin. With `use_tf:=false`, input coordinates are used directly
and the sensor origin is `(0,0,0)`.

The legacy `TransformStamped` topic path remains available through
`use_tf_topic`. It interpolates bracketed transforms and records nearest-time
fallbacks. Generic TF is preferred; the College preset explicitly retains its
legacy input topic. Neither mode deskews scans or reintegrates earlier data
after loop closure.

## Configuration and memory

Edit a preset under `config/`, or pass an independent YAML file with
`config_file:=/path/to/settings.yaml`. Mapping parameters are read-only after
startup; invalid configurations exit nonzero.

| Parameter | Behavior |
| --- | --- |
| `tdf_grid_res` | Positive voxel size in meters; 0.03 m and other nonreciprocal sizes are supported. |
| `tdfGridSize{X,Y,Z}_{low,high}` | Finite, ordered fixed-frame bounds; input upper bounds are exclusive. |
| `tdf_max_cells` | Maximum active blocks, allocated lazily. Blocks contain an integer number of voxels per side, approximately one meter across. |
| `capacity_policy` | `stop` preserves existing blocks and counts skipped allocations; `rolling` explicitly evicts blocks and counts evictions. |
| `memory_budget_mb` | Optional MiB limit on estimated full grid capacity plus kernels; 0 disables the guard. This is not a whole-process RSS limit. |
| `snapshot_budget_mb` | Optional MiB limit on a map snapshot; 0 disables the guard. Export scratch space is additional. |
| `num_threads` | 0 follows OpenMP settings; positive values select a worker limit. One owner writes each destination block. |
| `kernel_size`, `occ_min_hits` | Odd kernel edge 1–63; occupancy threshold 1–255. |
| `bins_az`, `bins_el`, `shadow_radius`, `distance_mode` | Direction bins, shadow radius in voxels, and `L1` or `L2` distance ranking. |
| `pc_downsampling`, `min_range`, `max_range` | Positive sampling stride and range limits measured from the sensor origin. |
| `cloud_reliability`, `cloud_queue_depth` | `best_effort` or `reliable`, with bounded KeepLast history. Live defaults: best effort / 10; launch replay defaults: reliable / 100. Explicit YAML wins. |
| `mesh_mode`, `mesh_smoothing_sigma` | `occupancy` keeps the established sign-based extraction; experimental `distance` uses signed metric ranks and requires observed cube corners. Sigma is in voxels; -1 selects 1 for occupancy and 0 for distance. |
| `allow_latest_tf`, `tf_timeout` | Zero timestamps are rejected by default; timestamped lookup timeout defaults to 0.1 s. |
| `legacy_max_skew`, `legacy_allow_nearest` | Legacy pose timing limit (0.1 s) and nearest-sample fallback (true). |
| `publish_cloud` | Publish the latest transformed input only when subscribed. |

A dense block-address table is retained, while voxel blocks are lazy. Large
bounds still cost table memory. `get_status` reports estimated capacity,
allocated bytes, blocks, skipped allocations, and evictions. Stopping new
allocations preserves older blocks; existing blocks continue to integrate.

Distance meshing remains an evaluation option. Its default disables smoothing;
positive smoothing can bias surfaces near unknown space. The `L2` lookup
converts distance ranks back to metric radii instead of treating ranks as meters.

## Status, exports, and map preview

Services use `std_srvs/srv/Trigger` and follow the launch namespace. Examples
below use the root namespace:

```bash
ros2 service call /get_status std_srvs/srv/Trigger '{}'
ros2 service call /save_grid_pcd std_srvs/srv/Trigger '{}'
ros2 service call /export_status std_srvs/srv/Trigger '{}'
ros2 service call /publish_map std_srvs/srv/Trigger '{}'
```

An accepted export returns a **job ID and output directory**, with state
`queued`. Poll `export_status` for `complete` or `failed` and its error. One
worker holds at most two pending/running jobs and retains the last 16 statuses.
It snapshots the map under a lock, then extracts and writes after releasing
the lock. Snapshot copying can still briefly pause integration. Completed
directories appear atomically beneath `output_directory` (default `maps`).

| Service | Result |
| --- | --- |
| `get_status` | Effective parameters, build provenance, received/integrated frames, TF failures, filtering, last timestamp, memory and capacity counters. Also published once per second on `~/status`. |
| `save_grid_pcd`, `save_grid_ply` | Occupied surface voxel centers in `grid_data.pcd` / `grid_data.ply`. Empty surfaces fail explicitly. |
| `save_grid_csv` | Per-block CSV with XYZ, rank, metric distance magnitude, free/observed flags and hit count. |
| `save_grid_mesh` | `mesh.stl`. Empty meshes and writer errors fail explicitly. |
| `save_grid_checkpoint` | Versioned `map.dbtsdf`, preserving voxel state and rolling eviction order. Resume with matching configuration and `initial_checkpoint`. |
| `publish_map` | Snapshot of the accumulated surface on transient-local `map_cloud`, with the snapshot's timestamp. No file is written. |
| `reset_map` | Clear the live map and counters, increment the map epoch; existing snapshots remain independent. |
| `save_grid_atak_zip` | OBJ, MTL and georeference sidecar in `atak_mesh.zip`; requires an explicit datum. |
| `get_geo_origin` | Immutable WGS-84/ENU datum and latest valid fix. |

Every file export includes `run.json` with the exact integrated frame count,
last timestamp, configuration, source fingerprint, canonical voxel state hash,
and SHA-256 output checksums. The `cloud` topic displays the latest input;
`map_cloud` displays the accumulated surface from `publish_map`.

For GNSS and ATAK setup, see [the frame and datum contract](docs/GEOREFERENCE_AND_ATAK.md).

## Datasets and reproducible evaluation

```bash
./scripts/download_test.sh                     # Convert MaiCity sequence 01
./scripts/download_mai_city.sh                 # Convert sequences 00, 01, 02
python3 scripts/benchmark.py --runner ./build/db_tsdf_offline \
  --output benchmark-results --threads 1 2 4 --repeats 3
```

Both download entry points fetch the **complete, multi-GB MaiCity archive**.
They retain a resumable cache, isolate the pinned `rosbags==0.10.11` converter,
record dependency versions and checksums, verify cached results, and preserve
existing output on failure. Install `python3-venv` and `wget` first. Pass
`--cache`, `--output`, or `--sha256 TRUSTED_DIGEST` as needed. Without a supplied
trusted digest, the first downloaded hash detects subsequent changes but does
not independently authenticate the archive.

The benchmark defaults to the download-free fixture and fails if voxel state
changes across repeats or worker counts within a backend. It reports latency
variation, memory, nearest-neighbor accuracy/completeness, and F-score at an
explicit tolerance. Per-run manifests retain full details. See
[reproducibility and the external input format](docs/REPRODUCIBILITY.md),
[upgrade notes](docs/CHANGES_1.1.md), and [local validation](docs/VALIDATION_1.1.md).

## Citation
If you use DB-TSDF in your research, please cite our paper:

```bibtex
@inproceedings{maese2026dbtsdf,
  title={DB-TSDF: Directional Bitmask-based Truncated Signed Distance Fields for Efficient Volumetric Mapping},
  author={Maese, Jose E. and Caballero, Fernando and Merino, Luis},
  booktitle={2026 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2026}
}
```

 


## Acknowledgements

![Logos](docs/media/fondos_proyectos.png)

This work was supported by the grants PICRA 4.0 (PLEC2023-010353), funded by the Spanish Ministry of Science and Innovation and the Spanish Research Agency (MCIN/AEI/10.13039/501100011033); and INSERTION (PID2021-127648OB-C31), funded by the "Agencia Estatal de Investigación - Ministerio de Ciencia, Innovación y Universidades" and the "European Union NextGenerationEU/PRTR".

# Upgrading to 1.1

This revision fixes overlapping OpenMP writes and nonreciprocal-resolution
indexing. Expect maps to differ from earlier racy runs. Input points retain
their multiplicity; independent destination-block workers now produce the
same canonical voxel state as a serial point-by-point reference.

Changes that affect existing workflows:

- Live launch defaults to wall time, best-effort input and a bounded queue.
  A bag supplied to launch selects simulated time and reliable input by
  default. Explicit YAML QoS settings take precedence. Manual playback needs
  `mode:=replay` and `ros2 bag play BAG --clock`.
- Generic TF is the node default. The College preset explicitly keeps legacy
  transform-topic mode. The legacy history is sorted, interpolates bracketed
  samples, and exposes nearest-sample fallback counts. Set
  `legacy_allow_nearest: false` to require matching or bracketed poses;
  this can reject live clouds that arrive before their future pose sample.
- Zero-stamp clouds in TF mode are rejected unless `allow_latest_tf: true`.
  Invalid XYZ layouts, quaternions, nonfinite points and bad configuration
  receive explicit validation. Startup failure exits nonzero. Mapping
  parameters cannot silently change without reconfiguring the backend.
- Blocks allocate lazily. `capacity_policy: stop` preserves the existing
  map and counts rejected new blocks. Select `rolling` explicitly for bounded
  eviction. Stop does not freeze updates to existing blocks. A scan touching
  more blocks than capacity is reported as incomplete under either policy.
- A block is approximately one meter across, with an integer number of
  voxels per side. At 0.03 m it has 34 voxels and spans 1.02 m. Blocks are
  anchored to the configured lower bounds. Input bounds are half-open;
  a partially clipped final voxel can have its center beyond the input bound.
- Surface exports consistently use voxel centers. Meshing uses neighbor
  halos and removes duplicate block-boundary ownership. `occupancy` remains
  the default extraction mode. `distance` is experimental, uses signed
  metric distances, and rejects cubes with unobserved corners. Positive
  smoothing may still bias geometry near unknown regions.
- Export services return a queued job ID and a unique directory under
  `output_directory`. Automation must poll `export_status` for completion.
  Empty PCD/PLY/mesh exports fail. Filenames within each directory remain
  familiar; root-level files are no longer overwritten.
- CSV exports contain one CSV per active block, with explicit observed/free
  flags and distance magnitude in meters. The old extra per-block PLY files
  are no longer emitted; use `save_grid_ply` for the surface.
- `cloud` contains the latest transformed input. Request `publish_map` for
  an accumulated snapshot on transient-local `map_cloud`. Services and map
  topics follow the launch namespace; absolute input/GPS topics stay absolute.
- Checkpoints use version `DBTSDF2` with explicit little-endian voxel fields,
  metadata/voxel corruption checks, and preserved eviction order. They require
  matching bounds, resolution, capacity, integration settings, mask width and
  frame/datum. A checkpoint load that fails leaves the existing map intact.
  They are not compatible with PCD/PLY or raw memory dumps. Core callers can
  provide any consistent frame signature; ROS includes its datum signature.
- `computeDistInterpolation` returns signed metric distances in local voxel
  coordinates. Check `valid`; unsupported queries return NaN from
  `interpolate`. The public coefficients are now local, double-precision
  values with an origin and inverse resolution.
- Docker builds the supplied source. Native CPU instructions are opt-in.
  Dataset scripts retain downloads, pin the converter, record dependencies,
  and verify existing results instead of deleting them. Both download
  entry points still fetch the complete archive.

The canonical state hash is FNV-1a over sorted observed voxel coordinates and
explicit state fields. It deliberately excludes allocation order, padding and
configuration. Compare it only alongside matching configuration/input hashes
and within a mask width. Output and input SHA-256 hashes provide artifact
integrity; they do not establish ground-truth accuracy.

Submaps, reintegration after pose corrections, dynamic-object aging, and
incremental mesh caching remain future work. Current fusion is monotonic and
cannot undo earlier observations. Presets have not been retuned from the
synthetic baseline.

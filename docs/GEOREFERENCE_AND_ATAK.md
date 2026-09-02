# DB-TSDF GPS and ATAK integration

DB-TSDF maps in a local Cartesian frame. A `NavSatFix` alone cannot make that
frame geographic: it supplies a position but not the fixed datum, ENU
orientation, or the time-aligned `map -> odom` transform needed for each
cloud. This package uses the following contract:

```text
WGS-84 fix + IMU + local odometry
             |
             v
robot_localization/navsat_transform_node --> odometry/gps
             |                                     |
             +--> robot_localization/ekf_node -----+
                              |
                         map -> odom
                              |
cloud frame -> ... -> odom -> map -> DB-TSDF
```

`earth` is recorded as the WGS-84 parent in exported metadata. The
`robot_localization` pipeline provides the operational `fix -> map -> odom`
relationship; it does not publish a generic ECEF `earth -> map` TF. Do not
substitute a UTM frame for `earth` without preserving its zone and hemisphere.

## Bring-up

1. Survey/select one WGS-84 datum: latitude, longitude, **ellipsoid** height,
   and ENU map yaw. Use the same datum for every component below; never
   silently choose the first live GNSS sample for a repeatable map.

2. Start the georeference pipeline after the local estimator is publishing
   `/odometry/local` and `odom -> base_link`:

   ```bash
   ros2 launch db_tsdf georeference_launch.py \
     datum:='34.12000000,-90.55000000,0.00000000' \
     gps_topic:=/gps/fix imu_topic:=/imu/data \
     local_odom_topic:=/odometry/local
   ```

   The third `robot_localization` datum value is yaw in radians, not altitude.
   Configure the surveyed ellipsoid height separately in DB-TSDF. For strict
   vertical-origin control, set the full `GeoPose` datum through the installed
   `navsat_transform_node` interface before mapping. The node also needs a
   valid IMU orientation and a TF from the GNSS antenna frame to `base_link`
   when they are different. Set magnetic declination and yaw offset for the
   installed sensor; default zero values are only correct for already-ENU IMU
   yaw.

3. Copy `config/atak_georeferenced.yaml`, set the surveyed latitude,
   longitude, ellipsoid height, and the **same yaw** used by the
   `robot_localization` datum, then change `geo_origin_configured` to `true`.
   Launch DB-TSDF with
   `fixed_frame_id:=map`. DB-TSDF will then look up `map <- cloud_frame` at the
   cloud timestamp before integrating.

   ```bash
   ros2 launch db_tsdf mapper_launch.py \
     config:=atak_georeferenced use_sim_time:=false
   ```

4. Connect the site TAK bridge at the ROS boundary. Feed incoming ATAK/GNSS
   positions as `sensor_msgs/NavSatFix` on `in_gps_topic`; DB-TSDF republishes
   only valid fixes on `out_gps_topic` with transient-local QoS. CoT encoding,
   identity, endpoint credentials, and map-file transfer remain bridge
   responsibilities, not DB-TSDF responsibilities.

## Exports

`/save_grid_atak_zip` refuses to run without an explicit datum. It creates an
OBJ/MTL plus `atak_mesh.origin.json`, whose schema is `db-tsdf-georef/v1`. The
sidecar states WGS-84 ENU, frame IDs, datum, altitude reference, yaw convention,
and the latest valid GNSS fix with its covariance. It is a bridge input contract; an OBJ plus a
private JSON sidecar is not by itself proof that a particular ATAK client will
import it as a native overlay.

`/get_geo_origin` returns the same JSON contract. Validate a deployment by
placing a known object at a surveyed location, exporting it, and checking both
its ATAK placement and the corresponding ROS `map` coordinates.

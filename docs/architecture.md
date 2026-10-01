# Architecture

## Overview

```
      ┌──────────────────── Gazebo Fortress (gz-sim 6) ────────────────────┐
      │   course.sdf  +  autobot (URDF spawned via ros_gz_sim `create`)     │
      │                                                                     │
      │  DiffDrive system   gpu_lidar    rgbd_camera     navsat (GPS)       │
      └───────┬────────────────┬─────────────┬────────────────┬───────────┘
              │  gz transport topics (gz.msgs.*)               │
      ┌───────▼────────────────────────────────────────────────▼──────────┐
      │                        ros_gz_bridge                                │
      │        gz ⇄ ROS 2 : /clock /cmd_vel /odom /tf /scan /gps/fix ...    │
      └───────┬────────────────┬─────────────┬────────────────┬───────────┘
       /odom(+TF)│        /scan │    /camera/*│         /gps/fix│
            ┌─────▼──────────────▼────────────┐          ┌──────▼────────┐
            │        waypoint_navigator        │          │ gps_localizer │
            │  go-to-goal + reactive avoidance │          │ NavSatFix→ENU │
            └─────┬────────────────────────────┘          └──────┬────────┘
           /cmd_vel│ (→ bridge → DiffDrive)                /gps/odom│
                  ▼                                                ▼
            robot moves in gz-sim                            RViz / logs
```

Everything is launched by `robot_bringup/launch/bringup.launch.py`, which
includes the Gazebo launch (gz-sim + `ros_gz_bridge` + spawn), the navigation
launch (after a short delay so the sensors are publishing), and RViz.

## Packages

| Package | Build type | Responsibility |
|---|---|---|
| `robot_description` | ament_cmake | URDF/Xacro, gz-sim sensor plugins, RViz config |
| `robot_gazebo` | ament_cmake | World (`course.sdf`), gz-sim bring-up + `ros_gz_bridge` + spawn |
| `robot_navigation` | ament_python | `waypoint_navigator`, `gps_localizer`, params, tests |
| `robot_bringup` | ament_cmake | Top-level single-command launch |

Separating description / simulation / behaviour / bring-up is the standard ROS 2
layout: the robot model is reusable on real hardware (swap `robot_gazebo` for
drivers), and the navigation package has no dependency on Gazebo.

## TF tree

```
odom                      (published by the DiffDrive system, bridged to /tf)
└── base_footprint
    └── base_link
        ├── left_wheel_link
        ├── right_wheel_link
        ├── caster_link
        ├── lidar_link            → /scan frame
        ├── camera_link
        │   └── camera_optical_link   → image/pointcloud frame (REP-103)
        └── gps_link              → /gps/fix frame
```

`robot_state_publisher` publishes the static transforms from the URDF (driven by
the bridged `/joint_states`); the gz-sim `DiffDrive` system publishes
`odom → base_footprint`, bridged to ROS `/tf` by `ros_gz_bridge`.

## Nodes and topics

### `waypoint_navigator` (robot_navigation)
- **Subscribes:** `/odom` (`nav_msgs/Odometry`), `/scan` (`sensor_msgs/LaserScan`, best-effort QoS)
- **Publishes:** `/cmd_vel` (`geometry_msgs/Twist`), `/planned_path` (`nav_msgs/Path`), `/waypoint_markers` (`visualization_msgs/MarkerArray`)
- **Params:** see `config/nav_params.yaml`

### `gps_localizer` (robot_navigation)
- **Subscribes:** `/gps/fix` (`sensor_msgs/NavSatFix`)
- **Publishes:** `/gps/odom` (`nav_msgs/Odometry`, local ENU)
- Sets the first valid fix as the datum and projects later fixes to local metres.

## Control algorithm

The navigator is a **reactive pure-pursuit controller** running at 20 Hz. It is
pure-arcing: it *always* keeps a forward velocity and steers as an arc, and
never pivots in place (a differential-drive robot steers well in motion but, in
gz-sim here, slips its wheels rather than rotating when stationary).

1. **Waypoint management.** Hold an ordered list of `(x, y)` goals in the `odom`
   frame. Advance to the next when within `goal_tolerance`; stop after the last.
2. **Go-to-goal.** Compute the bearing to the active waypoint; angular command is
   proportional, `kp_angular * heading_error` (clamped to `max_angular`). Forward
   speed scales with heading alignment and with front clearance, and eases down
   near the waypoint, but never drops to zero — so a large heading error is
   corrected as a tightening arc rather than a stop-and-spin.
3. **Reactive obstacle avoidance.** From the lidar, take the min range in a front
   cone (`±front_half_angle`) and in left/right side sectors.
   - `front < slow_distance` → scale forward speed down.
   - `front < avoid_distance` → override go-to-goal: arc toward the freer side.
     The turn direction is **latched** until the path clears, so it does not
     chatter when the two sides are similarly clear.

No prebuilt map or global planner is needed, which keeps the demo reproducible.
The usual reactive limitation applies (large concave/U-shaped traps), so the
course uses convex, separated obstacles.

**Simulation-fidelity note.** On a GPU-less, software-rendered rig the gz-sim
diff-drive has significant wheel slip during turns, so the model's ground-truth
pose drifts from the wheel odometry the controller navigates on. The navigation
logic, sensing and integration are all correct; closing the loop on a slip-free
or ground-truth-localised setup (e.g. a GPU host, or `robot_localization`) makes
the physical trajectory track the odometry one.

### Why not Nav2 here?

Nav2 (AMCL/SLAM + global/local planners) is the production-grade choice and
would also satisfy the task. It is intentionally **not** the default because:

- it requires a map (SLAM run or a static map) and non-trivial parameter tuning,
  which hurts "clone-and-run" reproducibility for a short assessment;
- the custom controller makes the navigation logic explicit and auditable.

To extend toward Nav2: add `slam_toolbox` for the map, `nav2_bringup` with a
tuned `nav2_params.yaml`, and replace `waypoint_navigator` with a
`FollowWaypoints` action client. The URDF, sensors, and TF tree here are already
Nav2-compatible (`base_footprint`, `/scan`, `odom`).

## GPS integration

The Gazebo world carries a `<spherical_coordinates>` georeference, so the navsat
sensor emits real WGS-84 lat/lon on `/gps/fix`. `gps_localizer` converts fixes
to a local ENU frame and publishes `/gps/odom`, logging the horizontal distance
to the goal. For a real deployment, wheel odometry and GPS would be fused with
`robot_localization` (two EKFs: continuous `odom→base_link`, and a
`navsat_transform_node` + global EKF for `map→odom`).

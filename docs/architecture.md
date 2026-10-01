# Architecture

## Overview

```
          ┌─────────────────────── Gazebo Classic ───────────────────────┐
          │  course.world  +  autobot (URDF spawned via spawn_entity.py)  │
          │                                                               │
          │  diff_drive plugin   ray sensor    depth camera    gps sensor │
          └───────┬───────────────────┬────────────┬──────────────┬──────┘
         /odom (+TF)│            /scan │   /camera/*│      /gps/fix │
                    │                  │            │               │
            ┌───────▼──────────────────▼────────────┐       ┌──────▼────────┐
            │        waypoint_navigator             │       │ gps_localizer │
            │  go-to-goal + reactive avoidance      │       │ NavSatFix→ENU │
            └───────┬───────────────────────────────┘       └──────┬────────┘
             /cmd_vel│                                        /gps/odom│
                    ▼                                                 ▼
             back to diff_drive plugin                          RViz / logs
```

Everything is launched by `robot_bringup/launch/bringup.launch.py`, which
includes the Gazebo launch, the navigation launch (after a short delay so the
sensors are publishing), and RViz.

## Packages

| Package | Build type | Responsibility |
|---|---|---|
| `robot_description` | ament_cmake | URDF/Xacro, Gazebo sensor plugins, RViz config |
| `robot_gazebo` | ament_cmake | World (`course.world`), Gazebo bring-up + robot spawn |
| `robot_navigation` | ament_python | `waypoint_navigator`, `gps_localizer`, params, tests |
| `robot_bringup` | ament_cmake | Top-level single-command launch |

Separating description / simulation / behaviour / bring-up is the standard ROS 2
layout: the robot model is reusable on real hardware (swap `robot_gazebo` for
drivers), and the navigation package has no dependency on Gazebo.

## TF tree

```
odom                      (published by the diff_drive plugin)
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

`robot_state_publisher` publishes the static transforms from the URDF; the
`diff_drive` plugin publishes `odom → base_footprint` and the wheel joints.

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

The navigator is a **hybrid reactive controller** running at 20 Hz:

1. **Waypoint management.** Hold an ordered list of `(x, y)` goals in the `odom`
   frame. Advance to the next when within `goal_tolerance` (0.25 m); stop after
   the last.
2. **Go-to-goal.** Compute the bearing to the active waypoint and a proportional
   angular command `kp_angular * heading_error` (clamped to `max_angular`).
   Forward speed scales with heading alignment and eases down as the robot nears
   the waypoint. If the heading error exceeds `heading_align` the robot rotates
   (near) in place to line up before driving forward.
3. **Reactive obstacle avoidance.** From the lidar, compute the minimum range in
   a front cone (`±front_half_angle`) and in left/right side sectors.
   - `front < slow_distance` → scale forward speed down.
   - `front < avoid_distance` → override go-to-goal: turn toward the side with
     greater clearance (gap following); creep forward only if `front >
     stop_distance`, otherwise rotate in place.

This needs no prebuilt map or global planner, which keeps the demo fully
reproducible. Its limitation is the usual one for purely reactive methods — it
can be trapped by large concave (U-shaped) obstacles; the course is designed
with convex, separated obstacles so a reactive policy reaches the goal reliably.

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

The Gazebo world carries a `<spherical_coordinates>` georeference, so the GPS
sensor emits real WGS-84 lat/lon on `/gps/fix`. `gps_localizer` converts fixes
to a local ENU frame and publishes `/gps/odom`, logging the horizontal distance
to the goal. For a real deployment, wheel odometry and GPS would be fused with
`robot_localization` (two EKFs: continuous `odom→base_link`, and a
`navsat_transform_node` + global EKF for `map→odom`).

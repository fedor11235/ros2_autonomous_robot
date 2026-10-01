# Recording the demo

The assessment asks for a **1–3 minute video** showing Gazebo + RViz, and an
optional rosbag. This is the checklist.

## 1. Launch

```bash
./run.sh            # or: docker compose up --build
```

Arrange the windows so both the Gazebo view and the RViz view are visible
(side by side). In RViz confirm these displays are on: RobotModel, LaserScan,
Odometry, PlannedPath, Waypoints.

## 2. Record a rosbag (optional but recommended)

In a second sourced terminal, before (or just as) the robot starts moving:

```bash
source install/setup.bash
ros2 bag record -o demo_bag \
    /tf /tf_static /odom /scan /cmd_vel \
    /gps/fix /gps/odom /planned_path /waypoint_markers \
    /camera/depth/image_raw
```

Stop with `Ctrl-C` once the robot reaches the goal. Replay:

```bash
ros2 bag play demo_bag
```

## 3. Screen-record the run (1–3 min)

- **Linux:** OBS Studio, or `ffmpeg`:
  ```bash
  ffmpeg -video_size 1920x1080 -framerate 30 -f x11grab -i :0.0 demo.mp4
  ```
- **macOS/Windows:** OBS Studio or the built-in screen recorder.

Suggested narration/beats:
1. Gazebo: show the arena, start (green) and goal (red) pads, obstacles.
2. RViz: show the lidar scan and the planned waypoint path.
3. Let the robot drive the full route — highlight it slowing/steering around
   each obstacle.
4. End on the robot stopped on the goal pad + the terminal line
   `Final waypoint reached. Goal complete.`

Keep it under 3 minutes; 2× speed for the straight segments is fine.

## 4. Verify the run was clean

```bash
ros2 topic hz /scan          # ~10 Hz
ros2 topic hz /odom          # ~50 Hz
ros2 topic echo /gps/fix --once
ros2 node list               # waypoint_navigator, gps_localizer present
```

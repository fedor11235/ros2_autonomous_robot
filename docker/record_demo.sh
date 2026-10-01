#!/usr/bin/env bash
# Headless demo recorder for machines WITHOUT a GPU, in two decoupled phases:
#
#   Phase 1  gz-sim runs HEADLESS with autonomy (no renderer stealing CPU, so the
#            reactive controller keeps correct timing) and a rosbag is recorded.
#   Phase 2  the bag is replayed into RViz and the RViz window is screen-captured.
#            Replay has no physics load, so software-GL (llvmpipe) renders the
#            already-correct run smoothly.
#
# This sidesteps the fact that a live Gazebo GUI or RViz under llvmpipe starves
# physics and breaks navigation. On a GPU machine just record `./run.sh` live.
#
# Env tunables:
#   RES      Xvfb resolution (default 1280x720)
#   MAX_WALL max seconds to wait for the goal in phase 1 (default 280)
set -e

RES="${RES:-1280x720}"
MAX_WALL="${MAX_WALL:-280}"
OUT="/out/demo.mp4"
BAG="/tmp/run_bag"
mkdir -p /out
rm -rf "$BAG"

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export QT_X11_NO_MITSHM=1
export DISPLAY=:1

RVIZ_CFG="$(ros2 pkg prefix robot_description)/share/robot_description/rviz/robot.rviz"

# ============================ Phase 1: headless sim + bag ============================
echo ">> Phase 1: headless gz-sim + navigation, recording rosbag ..."
Xvfb :1 -screen 0 "${RES}x24" -ac +extension GLX +render -noreset >/tmp/xvfb.log 2>&1 &
XVFB_PID=$!
sleep 3

ros2 launch robot_bringup bringup.launch.py gui:=false rviz:=false use_sim_time:=true \
  >/tmp/sim.log 2>&1 &
SIM_PID=$!
sleep 8

ros2 bag record -o "$BAG" \
  /clock /tf /tf_static /scan /odom /joint_states \
  /planned_path /waypoint_markers /gps/fix /cmd_vel >/tmp/bag.log 2>&1 &
BAG_PID=$!

echo ">> Waiting for the robot to reach the goal (max ${MAX_WALL}s) ..."
reached=0
for i in $(seq 1 "$MAX_WALL"); do
  if grep -q "Goal complete" /tmp/sim.log; then
    echo "   Goal reached after ~${i}s wall."
    reached=1
    break
  fi
  sleep 1
done
[ "$reached" = 0 ] && echo "   (timeout; recording whatever was navigated)"

sleep 2
kill -INT "$BAG_PID" 2>/dev/null || true    # finalize the bag cleanly
sleep 3
kill "$SIM_PID" 2>/dev/null || true
# Stop any lingering gz-sim server processes so they don't load phase 2.
pkill -f "ign gazebo" 2>/dev/null || true
pkill -f "ros_gz" 2>/dev/null || true
sleep 3

echo "---- phase 1 nav events ----"
grep -E "Reached waypoint|Goal complete" /tmp/sim.log || true

# ============================ Phase 2: replay into RViz + record ============================
echo ">> Phase 2: replaying bag into RViz and recording ..."
# robot_state_publisher provides the RobotModel + link TF during replay.
ros2 launch robot_description description.launch.py use_sim_time:=true >/tmp/rsp.log 2>&1 &
sleep 3
rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true >/tmp/rviz.log 2>&1 &
for i in $(seq 1 40); do
  wmctrl -l 2>/dev/null | grep -qiE 'rviz' && { echo "   RViz up after ${i}s."; break; }
  sleep 1
done
wmctrl -r RViz -b add,maximized_vert,maximized_horz 2>/dev/null || true
sleep 4

# Bag duration -> recording length.
DUR=$(ros2 bag info "$BAG" 2>/dev/null | sed -n 's/^Duration:[[:space:]]*\([0-9]*\).*/\1/p' | head -1)
DUR="${DUR:-120}"
REC=$((DUR + 6))
echo "   bag duration ~${DUR}s; recording ${REC}s of RViz."

ffmpeg -y -loglevel warning -video_size "$RES" -framerate 15 -f x11grab -i :1.0 \
  -t "$REC" -pix_fmt yuv420p -vcodec libx264 -preset ultrafast -crf 28 "$OUT" &
FF_PID=$!
sleep 1
ros2 bag play "$BAG" --clock 100 >/tmp/play.log 2>&1 || true
wait "$FF_PID" 2>/dev/null || true

kill "$XVFB_PID" 2>/dev/null || true
echo ">> Saved: $OUT"
ls -lh "$OUT" || true

#!/bin/bash
S=$SCRATCH
export MAMBA_ROOT_PREFIX=$S/mamba_root; eval "$($S/mm/bin/micromamba shell hook -s bash)"; micromamba activate $S/ros
export MPLBACKEND=Agg ROS_HOME=$S/ros_home ROS_LOG_DIR=$S/ros_home/log
mkdir -p $S/ros_home $S/runs
source $S/ws/devel/setup.bash
roscore > $S/runs/roscore.log 2>&1 &
RC=$!; for i in $(seq 1 30); do rostopic list >/dev/null 2>&1 && break; sleep 1; done; echo "roscore up after ${i}s"

run_case () {  # name left_type right_type seconds
  echo "================= CASE $1 (left cones type=$2, right cones type=$3)"
  rosparam delete /planner >/dev/null 2>&1
  timeout 40 roslaunch fs_planning-2022 path_planner_python.launch > $S/runs/$1_launch.log 2>&1 &
  LP=$!; sleep 4
  timeout 30 python $S/fake_world.py $2 $3 $4 $S/runs/$1.json > $S/runs/$1_world.log 2>&1
  sleep 1; kill -INT $LP 2>/dev/null; wait $LP 2>/dev/null
  echo "--- node output (filtered):"; grep -v -E "^\.\.\. logging|^Checking log|^Done checking|^started roslaunch|^ROS_MASTER_URI|^setting /run_id|^process\[|^SUMMARY|^=====|^PARAMETERS|^NODES|^auto-starting|^ \* |^$|^Press Ctrl|^\[path_planner-1\] process has finished" $S/runs/$1_launch.log | head -40
  echo "--- world summary:"; python - <<PY
import json; d=json.load(open("$S/runs/$1.json"))
print(" cone msgs sent:", d["cone_msgs_sent"], "| Path msgs received:", d["paths_received"], "| statuses:", sorted(set(d["statuses"])))
for s in d["samples"]: print("  ", s)
PY
}
run_case A_type0_left 0 1 12
run_case B_type1_left 1 0 12

echo "================= CASE C: node from the INSTALL space (params.yaml not installed)"
( source $S/ws/install/setup.bash; rosparam set /planner/map_topic /SLAM/cones; rosparam set /planner/waypoints_topic /planning/path; rosparam set /planner/use_cleaner true
  timeout 20 python $S/fake_world.py 1 0 15 $S/runs/C.json > /dev/null 2>&1 &
  sleep 2; timeout 12 rosrun fs_planning-2022 path_planner.py 2>&1 | grep -v "^\s*$" | tail -8; wait )

echo "================= CASE D: skidpad.py installed without a shebang, executed the way rosrun does"
( source $S/ws/install/setup.bash; timeout 5 $S/ws/install/lib/fs_planning-2022/skidpad.py 2>&1 | head -3; echo "exit=$?" )
echo "--- CASE D2: skidpad node via devel-space wrapper, headless"
( timeout 25 rosrun fs_planning-2022 skidpad.py > $S/runs/D2.log 2>&1 & sleep 1; T0=$(date +%s); timeout 20 rostopic echo -n1 /planning/path | grep -c "position" | sed 's/^/ poses with position fields: /'; echo " seconds until first path: $(( $(date +%s) - T0 ))"; wait; grep -v "^\s*$" $S/runs/D2.log | head -5 )
kill -INT $RC; wait $RC 2>/dev/null; echo "done"

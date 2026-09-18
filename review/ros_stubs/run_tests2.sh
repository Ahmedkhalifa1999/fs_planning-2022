#!/bin/bash
S=$SCRATCH
export MAMBA_ROOT_PREFIX=$S/mamba_root; eval "$($S/mm/bin/micromamba shell hook -s bash)"; micromamba activate $S/ros
export MPLBACKEND=Agg ROS_HOME=$S/ros_home ROS_LOG_DIR=$S/ros_home/log
source $S/ws/devel/setup.bash
roscore > $S/runs/roscore2.log 2>&1 &
RC=$!; for i in $(seq 1 30); do rostopic list >/dev/null 2>&1 && break; sleep 1; done
strip() { sed -r 's/\x1b\[[0-9;]*[a-zA-Z]//g; s/\x1b\]2;[^\x07]*\x07?//g'; }
run_case () {  # name left_type right_type seconds
  echo "================= CASE $1 (left cones type=$2, right cones type=$3) — world/TF started BEFORE the node"
  timeout 40 python $S/fake_world.py $2 $3 $4 $S/runs/$1.json > $S/runs/$1_world.log 2>&1 &
  WP=$!; sleep 3
  timeout 30 roslaunch fs_planning-2022 path_planner_python.launch > $S/runs/$1_launch.log 2>&1 &
  LP=$!; wait $WP; kill -INT $LP 2>/dev/null; wait $LP 2>/dev/null
  echo "--- node output (filtered):"; strip < $S/runs/$1_launch.log | grep -v -E "^\.\.\. logging|^Checking log|^Done checking|^started roslaunch|^ROS_MASTER_URI|^setting /run_id|^process\[|^SUMMARY|^=====|^PARAMETERS|^NODES|^auto-starting|^ \* |^$|^Press Ctrl|process has finished|^  /|^    path_planner|^/tmp|localhost:11311|^done|shutting down|^\[path_planner-1\] killing|^all processes" | head -30
  echo "--- world summary:"; python - <<PY
import json; d=json.load(open("$S/runs/$1.json"))
print(" cone msgs sent:", d["cone_msgs_sent"], "| Path msgs received:", d["paths_received"], "| statuses:", sorted(set(d["statuses"])))
for s in d["samples"]: print("  ", s)
PY
}
run_case A2_type0_left 0 1 14
run_case B2_type1_left 1 0 14
echo "================= CASE E: cleaner disabled (planner/use_cleaner false), type1 left"
rosparam set /planner/use_cleaner false
timeout 40 python $S/fake_world.py 1 0 10 $S/runs/E.json > /dev/null 2>&1 & WP=$!; sleep 3
( timeout 30 roslaunch fs_planning-2022 path_planner_python.launch 2>&1 | strip > $S/runs/E_launch.log ) & LP=$!
sleep 1; rosparam set /planner/use_cleaner false; wait $WP; kill -INT $LP 2>/dev/null; wait $LP 2>/dev/null
python - <<PY
import json; d=json.load(open("$S/runs/E.json")); print(" Path msgs received:", d["paths_received"]); [print("  ", s) for s in d["samples"]]
PY
grep -E "Traceback|Error" $S/runs/E_launch.log | head -3
kill -INT $RC; wait $RC 2>/dev/null; echo "done"

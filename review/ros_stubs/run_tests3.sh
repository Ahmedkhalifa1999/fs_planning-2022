#!/bin/bash
S=$SCRATCH
export MAMBA_ROOT_PREFIX=$S/mamba_root; eval "$($S/mm/bin/micromamba shell hook -s bash)"; micromamba activate $S/ros
export MPLBACKEND=Agg ROS_HOME=$S/ros_home ROS_LOG_DIR=$S/ros_home/log
source $S/ws/devel/setup.bash
roscore > $S/runs/roscore3.log 2>&1 & RC=$!; for i in $(seq 1 30); do rostopic list >/dev/null 2>&1 && break; sleep 1; done
for CASE in "B3 1 0 20" "A3 0 1 20"; do set -- $CASE
  rosparam set /planner/use_cleaner true
  timeout 45 python $S/fake_world.py $2 $3 $4 $S/runs/$1.json > /dev/null 2>&1 & WP=$!; sleep 3
  timeout 40 roslaunch fs_planning-2022 path_planner_python.launch > $S/runs/$1_launch.log 2>&1 & LP=$!
  wait $WP; kill -INT $LP 2>/dev/null; wait $LP 2>/dev/null
  python - <<PY
import json; d=json.load(open("$S/runs/$1.json")); tl=d["timeline"]
print("=== CASE $1 (left type=$2): paths received:", len(tl), "| empty:", sum(1 for _,n in tl if n==0), "| non-empty:", sum(1 for _,n in tl if n>0))
print("  first path at t=%.1fs" % tl[0][0] if tl else "  no paths")
fe = d["first_nonempty"]; print("  first NON-EMPTY path at t=%.1fs with %d poses, first pts %s" % (fe["t"], fe["n"], fe["pts"]) if fe else "  never non-empty")
gaps = [ (a[0], b[0]) for a,b in zip(tl, tl[1:]) if (a[1]>0) != (b[1]>0)]; print("  transitions empty<->non-empty at:", [round(g[1],1) for g in gaps][:10])
PY
done
kill -INT $RC; wait $RC 2>/dev/null

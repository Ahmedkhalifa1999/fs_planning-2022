# Review tooling

Supporting material for `../REVIEW.md`. Nothing in this directory is part of the ROS package; it exists so the findings can be reproduced.

## Layout

| Path | What it is |
|---|---|
| `harness/harness.py` | Runs `planner.Planner` on synthetic tracks with cone type `0` on the left. Prints chosen waypoints, cost and timing. |
| `harness/harness_swapped.py` | Same with cone type `1` on the left (the convention the current code actually expects). |
| `harness/microtests.py` | One isolated reproduction per planner bug listed in the review (FOV filter, triangulation distance, heading wrap, slope division, angle-change wrap, `log(0)`, shared class attribute, `get_cost` arity). |
| `harness/cleaner_harness.py` | Checks on `SplineInterpolator`, `ArcLengthSpline`, `WaypointsCleaner`, and a headless run of `skidpad.py`'s geometry. |
| `stubs_pkg/tf_helper/` | Minimal non-ROS stand-in providing `mutex_lock` and `np`, the only names `planner/planner.py` needs from `tf_helper`. |
| `ros_stubs/asurt_msgs/` | Stand-in catkin message package. `Landmark.msg` fields are reconstructed from how this repo and the ROS 2 `tf_helper` use them; the real definition is in a private repository. |
| `ros_stubs/tf_helper/` | Stand-in catkin package reconstructing the ROS 1 `tf_helper` API (`TF_Helper`, `Status_Publisher`, `mutex_lock`, `create_path_message`, `create_landmark_message`). Upstream now only has a ROS 2 version. |
| `ros_stubs/fake_world.py` | Publishes a straight synthetic track on `/SLAM/cones`, broadcasts `map -> base_link` (car moving at 1 m/s), `base_link -> rear_link`, `base_link -> kit`, and records `/planning/path`. Usage: `python fake_world.py <left_type> <right_type> <seconds> <out.json>`. |
| `ros_stubs/run_tests*.sh` | The runtime scenarios from the review. They expect `$SCRATCH` to contain `ws/` (catkin workspace), `ros/` (RoboStack env) and `mm/bin/micromamba`. |
| `logs/` | `startup_crash_no_tf.log` (node dying at startup), `A3.json` (type 0 left: mostly empty paths), `B3.json` (type 1 left: working), `E.json` (cleaner disabled). |

## Running the pure-Python parts

```bash
pip install numpy scipy matplotlib pyyaml
python review/harness/microtests.py
python review/harness/harness_swapped.py
python review/harness/cleaner_harness.py
```

## Building with ROS Noetic

```bash
mkdir -p ws/src
cp -r review/ros_stubs/asurt_msgs review/ros_stubs/tf_helper ws/src/
ln -s "$PWD" ws/src/fs_planning-2022
cd ws
catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCATKIN_WHITELIST_PACKAGES="asurt_msgs;tf_helper"
catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCATKIN_WHITELIST_PACKAGES=""
catkin_make install -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

The two-pass build is required because `package.xml` does not declare `tf_helper` as a build dependency, so catkin configures this package before `tf_helper` exists. On RoboStack environments `pip install empy==3.3.4` is also needed for message generation.

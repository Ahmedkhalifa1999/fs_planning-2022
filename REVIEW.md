# Codebase review: fs_planning-2022

Review date: 2026-09-18. Reviewed commit: `80557ac` ("Color bug fix", 2022-06-28), branch `master`.

## 1. Verdict

This is a small (about 1,060 lines of Python) ROS 1 path-planning package for a Formula Student driverless car, written between February and June 2022 by three contributors and untouched since. The core idea is sound and the planner is fast enough, but the repository as it stands **cannot be built or run from its own contents**, and once the missing pieces are stubbed in, the code has **several verified correctness bugs**, **no tests**, **no CI**, and a **README that describes a different design from the one implemented**.

The most important findings, in order:

1. **Unbuildable as published.** It depends on two external packages that are not pinned, vendored, or documented: `asurt_msgs` (private, unavailable) and `tf_helper`, whose upstream `main` branch has since been rewritten for ROS 2 and no longer exports any of the six names this repo imports. There is no branch or tag preserving the ROS 1 API.
2. **Packaging bugs that break `catkin_make` and the install space** even when the dependencies exist: `tf_helper` is required at CMake configure time but declared only as an exec dependency, so catkin builds this package before it; `params.yaml` is never installed, so the node throws `FileNotFoundError` from an install space; `skidpad.py` has no shebang and fails with `import: not found` when executed; the package name contains a hyphen, which REP-144 forbids; and `cmake_minimum_required(VERSION 3.0.2)` is rejected outright by CMake 4.
3. **Runtime crash at startup** if the TF tree is not yet available when the node starts (unchecked `None` at `src/nodes/path_planner.py:42`).
4. **Cone-colour convention is inverted relative to the package's own constants.** `planner.py` treats type `1` as the left-hand colour while `consts.py` declares `BLUE_CONE_STYLE = 0`. Under the `consts.py` convention the node publishes empty paths almost continuously. This must be checked against the real `Landmark.msg`.
5. **Verified logic bugs in the planner**: the field-of-view filter is one-sided (it keeps everything on the left half-plane, including behind the car); the Delaunay edge-length check uses `x` twice and is therefore orientation-dependent; heading averaging has no wrap-around (170° and -170° average to 0°); a slope computation raises `ZeroDivisionError` for a specific backwards step and biases every leftward step; and a colour probability of exactly 1.0 produces a cost of `-inf`.
6. **Waypoint cleaner** contains unreachable code referencing a parameter that does not exist, four unused parameters, unbounded memory growth, an unguarded spline fit that raises on duplicate points, and an arc-length evaluator that indexes out of range for short inputs.

Everything above was reproduced in this session. Section 9 explains how.

## 2. What the repository contains

| Path | Role | Lines |
|---|---|---|
| `src/nodes/path_planner.py` | ROS node: subscribes to cones, runs planner and cleaner, publishes `nav_msgs/Path` | 77 |
| `src/nodes/skidpad.py` | ROS node: publishes a hard-coded skidpad path once | 58 |
| `src/planner/` | Library: `Map` (Delaunay waypoint generation + beam search), `Path` (cost), `Waypoint`, `Cone`, `consts` | 496 |
| `src/waypoints_cleaner/` | Library: multi-frame waypoint memory, ordering, spline smoothing, arc-length resampling, `params.yaml` | 416 |
| `launch/` | Two launch files | |
| `CMakeLists.txt`, `package.xml`, `setup.py` | catkin packaging | |
| `README.md` | Design notes (stale) | |

Data flow: `LandmarkArray` on `/SLAM/cones` → transformed to `base_link` → `Map` filters cones within 10 m, builds Delaunay midpoints plus a ring of 8 candidate points around every cone → breadth-first beam search of at most 3 steps, scored by heading change, track-width deviation and colour likelihood → best path → optional `WaypointsCleaner` that accumulates waypoints in the `map` frame across frames and fits a smoothed spline → `nav_msgs/Path` republished in the `kit` frame.

## 3. How the review was done

- Read every file; ran `pyflakes`, `pylint`, `mypy`, `vulture` and `catkin_lint` (strict, with the noetic rosdep index).
- Built a Python harness with a stub `tf_helper` to drive `Planner` and `WaypointsCleaner` on synthetic tracks (straight, arc, degenerate, noisy) and wrote micro-tests for each suspected bug.
- Installed ROS Noetic (RoboStack via micromamba, Python 3.11, Ubuntu 24.04), wrote stand-in `asurt_msgs` and `tf_helper` ROS 1 packages reconstructed from usage, and built the workspace with `catkin_make` and `catkin_make install`.
- Ran the node under `roscore` with a synthetic world (moving car TF, cone stream at 10 Hz, `rear_link` and `kit` frames) and recorded `/planning/path` under several configurations.

All scripts, stubs and captured logs are in `review/`.

## 4. Build, dependency and packaging findings

### 4.1 Dependencies (blocking)

| Finding | Evidence |
|---|---|
| `tf_helper` is imported with `from tf_helper import *` and provides `TF_Helper`, `Status_Publisher`, `mutex_lock`, `create_path_message`, `create_landmark_message` and, implicitly, `np`. The upstream repository `asurt-fsai/tf_helper` has a single branch, `main`, which is now an `ament_python` ROS 2 package using `rclpy`, with classes `TFHelper`, `StatusPublisher`, `Utils`. None of the six names exist. | `git ls-remote` shows only `main`; `package.xml` there declares `<build_type>ament_python</build_type>` and `<depend>rclpy</depend>` |
| `asurt_msgs` (provides `LandmarkArray`) is private or gone. Its message layout can only be inferred from usage (`landmark.position.x/y`, `landmark.type`). | clone attempt: `could not read Username for 'https://github.com'` |
| Neither dependency is pinned by tag, submodule, `.rosinstall`, `rosdep` key or README instruction. | |

Without vendoring or pinning, nobody can rebuild the code that was on the car in 2022. If this repository is meant to be a historical record, the exact `tf_helper` and `asurt_msgs` revisions should be recorded (or copied in). If it is meant to be maintained, the six helper functions are small enough to inline.

### 4.2 catkin / CMake / install (verified by building)

| # | Finding | Where | Observed effect |
|---|---|---|---|
| B1 | `tf_helper` is in `find_package(catkin REQUIRED COMPONENTS ...)` but `package.xml` lists it only as `exec_depend`. catkin's topological sort therefore orders `fs_planning-2022` **before** `tf_helper`. | `CMakeLists.txt:10`, `package.xml:21` | `catkin_make` fails: `Could not find the required component 'tf_helper'`. Build log shows order `asurt_msgs, fs_planning-2022, tf_helper`. Workaround was a two-pass build. |
| B2 | `cmake_minimum_required(VERSION 3.0.2)` | `CMakeLists.txt:1` | CMake 4.x: `Compatibility with CMake < 3.5 has been removed`. Needed `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`. |
| B3 | Package name `fs_planning-2022` contains a hyphen. REP-144 requires `[a-z][a-z0-9_]*`. | `package.xml:3`, `CMakeLists.txt:2` | Works for a pure-Python package today, but breaks the moment a message, service or C++ target is added (Python module names cannot contain `-`). `setup.py` already silently mangles it to `fs_planning_2022`. |
| B4 | `src/waypoints_cleaner/params.yaml` is loaded relative to `__file__` but `setup.py` declares no `package_data`, so it is not installed. | `waypoints_cleaner.py:30-35`, `setup.py` | From the install space: `FileNotFoundError: .../site-packages/waypoints_cleaner/params.yaml`. Only works from a devel space. |
| B5 | `src/nodes/skidpad.py` has no shebang line. `catkin_install_python` installs it as an executable anyway. | `src/nodes/skidpad.py:1` | Executing it as `rosrun` would: `skidpad.py: 1: import: not found`. `catkin_lint` reports `missing_shebang`. |
| B6 | `setup.py` lists `nodes` as a Python package, but `src/nodes` has no `__init__.py`. The node scripts end up installed twice: `lib/fs_planning-2022/` and `site-packages/nodes/`. The names `nodes`, `planner`, `waypoints_cleaner` are also installed as top-level modules, which will collide with any other workspace package that picks the same generic names. | `setup.py:9` | Confirmed in `install/lib/python3.11/site-packages/`. |
| B7 | `path_planner.py` hard-codes `#!/usr/bin/python3`, so the install-space copy ignores the interpreter catkin was configured with (matters for venv/RoboStack/conda setups). | `src/nodes/path_planner.py:1` | Installed file keeps `#!/usr/bin/python3`. |
| B8 | `package.xml` declares `roscpp` (unused, there is no C++), and omits real runtime dependencies: `nav_msgs`, `geometry_msgs`, `tf`, `tf2_ros`, `asurt_msgs`, `python3-numpy`, `python3-scipy`, `python3-matplotlib`, `python3-yaml`. `catkin_package()` has no `CATKIN_DEPENDS`. | `package.xml`, `CMakeLists.txt:106` | `catkin_lint`: `package 'roscpp' must be in CATKIN_DEPENDS`, `unknown package 'tf_helper'`. `rosdep install` cannot set up a machine for this package. |
| B9 | `package.xml` metadata is template filler: empty maintainer name, `asurt@todo.todo`, version `0.0.0`, description "The fs_planning-2022 package". `CMakeLists.txt` is about 190 lines of untouched template comments around 10 lines of content. | | `catkin_lint`: `meaningless package description`. |
| B10 | An in-tree build leaves `src/fs_planning_2022.egg-info/` behind and `.gitignore` does not cover `*.egg-info`. | `.gitignore` | Appeared after `catkin_make`. |
| B11 | 16 of the 20 tracked files are mode `755`, including `README.md`, `package.xml`, `params.yaml` and the launch files. `consts.py` has CRLF line endings; seven files lack a trailing newline. `__pycache__` directories were committed 26 times in history before being cleaned up. | `git ls-files -s` | `catkin_lint` reports 15 `uninstalled_script` errors for the spurious executable bits. |

### 4.3 No quality gates

There are no unit tests, no `rostest`, no CI workflow, no linter configuration, no `requirements.txt`, and no pre-commit hooks. Static tools run cold on the tree give:

| Tool | Result |
|---|---|
| `pyflakes` | 53 messages (unused imports, star-import shadowing, undefined `plt`) |
| `pylint -E -W` | 6.55/10 |
| `mypy` | 14 errors (inconsistent return types, invalid annotations, undefined names) |
| `catkin_lint --strict` | 21 problems |
| `vulture` | unreachable code, unused parameters and constants |

## 5. Runtime findings (node under roscore)

All runs use the stand-in `tf_helper`, so behaviours that depend on how the real helper handled failures are marked as such.

| # | Scenario | Result |
|---|---|---|
| R1 | Launch node before the TF tree exists. | Node dies immediately: `TypeError: 'NoneType' object is not subscriptable` at `path_planner.py:42` (`cleaner = WaypointsCleaner(to_rear[0])`). The same `get_transform` call is `None`-checked at line 61 but not here. *(Depends on the helper returning `None` on lookup failure; with a helper that blocks forever the node would instead hang.)* No `respawn` in the launch file. |
| R2 | Straight track, TF up, cone type `0` on the left, `1` on the right (the `consts.py` convention: `BLUE_CONE_STYLE = 0`). | 159 `Path` messages in 20 s, **152 of them empty**. Non-empty paths appear only as flickers when the synthetic cone pattern wraps. |
| R3 | Same, but type `1` on the left. | 157 of 159 paths non-empty; first usable path 0.2 s after node start; about 100 poses at 0.1 m spacing, starting 1.5 m **behind** the `kit` origin (the cleaner prepends the rear-axle and current positions). |
| R4 | Cleaner disabled (`planner/use_cleaner=false`), type `1` left. | Correct 4-pose paths straight down the centreline, e.g. `[-0.5,0],[1.5,0],[2.75,0],[4.0,0]` in `kit`. Look-ahead is only 4 to 6.4 m. |
| R5 | Install space instead of devel space. | `FileNotFoundError` for `params.yaml` (B4). |
| R6 | `skidpad.py` from the install space, executed as `rosrun` would. | `import: not found` (B5). Via the devel-space wrapper, headless with `MPLBACKEND=Agg`, it publishes 187 poses after about 7 s. With a display it would first run a 1.9 s `plt.pause` animation; without one and without `MPLBACKEND` set it would raise. |

Other observations from the runs:

- The node publishes an **empty `Path`** on every cycle in which the cleaner has nothing, rather than skipping. Downstream code must cope with zero-pose paths.
- `status.running()` is only called after a path was published, so a watchdog sees no heartbeat while the planner returns `None`.
- Planning time on this VM: 13 to 25 ms for 16 cones, 60 to 73 ms for 100 cones. The loop is nominally 20 Hz (50 ms), and the cone callback is blocked by the same mutex while planning runs.

## 6. Planner logic: verified bugs

Each row was reproduced with `review/harness/microtests.py` on the current code.

| # | Location | Bug | Evidence | Impact |
|---|---|---|---|---|
| P1 | `src/planner/map.py:105-106` | Field-of-view test applies `abs()` **before** the modulo and never takes `abs()` of the final angle, so any negative result passes. | With `field_of_view=180`, cones at 0°, 45°, 90°, 135°, **180°** and -45° are kept; -90° and -135° are dropped. | Waypoint expansion (`WAYPOINT_FIELD_OF_VIEW = 180`) can step backwards or hard-left but not hard-right. Harness runs show best paths containing points at negative `x`. |
| P2 | `src/planner/map.py:132` | `dist = sqrt((x-l1.x)**2 + (x-l1.y)**2)` uses `x` where `y` belongs. | A vertical 6 m edge gets `dist = 0` (accepted); the same edge rotated horizontal gets `4.24` (rejected). | Which Delaunay midpoints become candidate waypoints depends on track orientation, not edge length. |
| P3 | `src/planner/waypoint.py:16-20` | Arithmetic mean of two angles, then wrap by subtracting `pi` instead of `2*pi`. | `avg(170°, -170°) = 0°` (should be 180°). | Wrong headings feed `get_largest_angle_change`, which drives the cost. |
| P4 | `src/planner/path.py:86-89` | `if denom < 1e-4: denom += 1e-4` treats every negative `dx` as "near zero": it biases the slope for any leftward or backward step and is exactly zero when `dx == -1e-4`. | `dx=-1e-4` → `ZeroDivisionError` (uncaught; the main loop has no exception handling, so the node dies). `dx=-1` gives slope `-1.0001`. Vertical segments get slope `1e4`. | The whole slope/intercept formulation is fragile; a vector projection is the standard fix. |
| P5 | `src/planner/path.py:55-66` | Heading differences are not wrapped to [-π, π]. | Two nearly collinear backward segments (179° then -179°) give a "largest change" of 358.9° and a +100 penalty. | Interacts with P1: backward expansions are allowed by the filter and then punished by an artefact. |
| P6 | `src/planner/path.py:128` | `np.log(1 - p)` with `p == 1.0`. | `cost = -inf`, `RuntimeWarning`. | One perfectly confident cone makes the cost of every path on that side `-inf`; `argmin` then picks arbitrarily. Probabilities need clamping. |
| P7 | `src/planner/planner.py:13-16` vs `src/planner/consts.py:1-5` | Type `1` maps to `[0.8,0.1,0.1]` (index 0 = the colour `color_cost` expects on the **left**), type `0` to `[0.1,0.8,0.1]`. `consts.py` says `BLUE=0, YELLOW=1` and those constants are never used. Formula Student puts blue on the left. | Runtime R2 vs R3. | Either `consts.py` or `planner.py` is wrong. Needs checking against `asurt_msgs/Landmark.msg`; should use the message constants instead of literals. |
| P8 | `src/planner/path.py:169-211` | `get_cost` returns a 3-tuple on the sharp-turn branch and a 7-tuple otherwise; the early return omits the colour term. `mypy` flags the declared `-> float`. | `get_cost()[3:5]` is `()` on the short branch. | Fragile callers; the `verbose` debugging path would misbehave. |
| P9 | `src/planner/map.py:58` | `deepcopy(current_path)` for every candidate expansion copies the full cone array and probability matrix (shared, immutable data) each time. | Profile: `deepcopy` is 33% of planner time (0.30 of 0.89 s over 20 runs). | Easy 2x speed-up; matters at 100 cones (70 ms > 50 ms budget). |
| P10 | `src/planner/map.py:139-140` | `except: pass` around the whole triangulation. | | Swallows the expected `QhullError` for fewer than 3 or collinear cones, but also any real bug. Also silently hides P2. |
| P11 | `src/planner/map.py:160-165` | `edge_in_edges` is missing `@staticmethod`, has invalid annotations `list()`/`tuple()`, and is an O(E²) scan. | `mypy`: `Invalid type comment or annotation`. | Should be a set of sorted tuples. |
| P12 | `src/planner/cone.py:5-7` | `x`, `y`, `color_probs` are mutable **class** attributes; `landmark != None`. | Mutating one instance's default list mutates all. | Latent aliasing bug; `Cone` is only half-used (the planner builds its own `Landmark` class instead). |
| P13 | `src/planner/path.py:104-106` | Projection is accepted if it lies within the segment's x-range **or** y-range (`logical_or`). | | Almost certainly meant `and`; as written the "distance to segment" is really "distance to line" for most cones. Undocumented fallback to endpoint distance otherwise. |
| P14 | `src/planner/consts.py` | `LENGTH_WEIGHT` assigned twice (second wins), five `*_CONE_STYLE` and `MAX_DIST_WEIGHT` unused, CRLF endings. Magic numbers in code: `2.5` rad, `100`, `+2` m, `1e-4`, `3.14` for π (twice). | | Tuning is spread across `consts.py` (code), `params.yaml` (file) and rosparams (launch): three mechanisms for one node. |
| P15 | `src/planner/map.py:37,46` and `consts.py:11-13` | Horizon is 3 search iterations of one waypoint each. | R4: 4 to 6.4 m look-ahead. | At 10 to 15 m/s that is under half a second of path. The cleaner's memory partly compensates, which is presumably why it exists. |
| P16 | `src/planner/waypoint.py:31-32` | `__eq__` on float equality without `__hash__`; used for `waypoint not in current_path.waypoints`. | | Works by accident because the same `Waypoint` objects are reused; fragile. |

## 7. Waypoint cleaner and spline code

| # | Location | Finding | Evidence |
|---|---|---|---|
| C1 | `waypoints_cleaner.py:182-200` | Code after `return all_waypoints` is unreachable and references `self.exp_param`, which is not in `params.yaml` (would raise `AttributeError` if ever reached). The docstring promises `num_waypoints_return` waypoints; the function returns about 100. | `vulture`: unreachable code (100% confidence). Harness: output shape `(99, 2)`. |
| C2 | `params.yaml` | `num_waypoints_return`, `num_frames_use`, `theta_lookahead`, `plot` are loaded but never used. | Harness check. |
| C3 | `waypoints_cleaner.py:66-69, 152` | `prev_done_waypoints` gets the same index appended on every tick the car is within 1 m of a waypoint; nothing is ever pruned from it or from `prev_waypoints`; membership is a list scan. | After 30 m: 14 stored waypoints, 26 "done" entries. Grows for the whole session; O(n²) per tick. |
| C4 | `waypoints_cleaner.py:62-63` | `reset()` wipes all memory whenever consecutive poses differ by more than 2 m. | Harness E: a 3 m jump empties the store. | Triggers on a SLAM loop-closure correction, or simply when the planner produced no path for a few cycles at speed (the cleaner is only updated when a path exists). |
| C5 | `waypoints_cleaner.py:109, 132` | The ordered list always starts with `rear_pos, curr_pos`; if `to_rear` is zero these duplicate and `splprep` raises `ValueError: Invalid inputs`. No try/except. | Harness E reproduces the exception. | Uncaught in the node loop: crash. |
| C6 | `waypoints_cleaner.py:125,132` | `k = 2` (quadratic) with the comment "Cubic spline"; smoothing `s=10` is independent of the number of points. | | |
| C7 | `spline_interpolator.py:122-140` | `evaluate` assumes knots at exact multiples of `delta_arclength`, but `fit_spline` places them at the cumulative estimated arclength (always at or beyond the multiple). | Circle r=5: knot deviation up to 0.027 m, radial error up to 0.025 m. Acceptable here, but wrong in principle. | `get_points` then samples at `arange(0, total, dt)` over the **first** spline's length, so requested 0.1 m spacing comes out as 0.049 to 0.58 m in the skidpad path. |
| C8 | `spline_interpolator.py:126-129` | When the computed interval is out of range the code **prints** three numbers and then indexes anyway. | 2- or 3-point inputs: `IndexError: index 9 is out of bounds`. | `get_waypoints` happens to avoid this (it always passes 20 interpolated points), `skidpad.py` too; the class itself is unsafe. |
| C9 | `spline_interpolator.py:29` | Bounds check is `interval > shape[1]` (off by one), and `find_interval` is a linear scan. | `evaluate(3.5)` on knots `[0..3]` silently extrapolates. | `scipy.interpolate.CubicSpline` is already constructed at line 16 and could simply be called. |
| C10 | `spline_interpolator.py:116-120` | `plt` is used but never imported. | `pyflakes`: undefined name `plt` (5x). | `plot=True` raises `NameError`. |
| C11 | `waypoints_cleaner.py:7` | `matplotlib.pyplot` imported at module import time on the vehicle, for a plotting path that is dead (C1). | | Slow import; may try to open a GUI backend. |
| C12 | `waypoints_cleaner.py:11-17` | A second class named `Waypoint`, unrelated to `planner.waypoint.Waypoint`. | | Confusing; both get pulled into the node's namespace by star imports. |

What works: with TF and colours right, the cleaner does produce a smooth centreline within 0.2 s, and on a straight track the output tracks the true centreline to within a few centimetres.

## 8. Node scripts, launch files, docs and process

### `src/nodes/path_planner.py`

- Unused imports `tf`, `time`, `tf2_ros`, `Odometry`; `global` declarations in a function that never assigns them; three star imports. `from planner import *` injects the submodule `map`, shadowing the built-in.
- Frames `base_link`, `rear_link`, `map`, `kit` and the topic `planner/possible_waypoints` are hard-coded; `rospy.get_param` is called without defaults, so a launch file that omits a key raises `KeyError`.
- `get_message_in` results are used unchecked at lines 19 and 70; `get_transform` is unchecked at line 41 (R1) but checked at line 61.
- The main loop has no exception handling and the launch file has no `respawn`, so any of P4, C5 or the transform issues takes the node down permanently.
- Possible-waypoint markers are published with type `4` ("unknown") hard-coded rather than the message constant.

### `src/nodes/skidpad.py`

- All logic runs at import time, so it cannot be imported or tested; no shebang (B5); `plt.pause` animation before publishing (R6); hard-coded geometry; the launch file names the node `path_planner` while the script calls `init_node("skidpad_pub")`.

### Launch files

- `use_sim_time` is forced to `false` as a global parameter by a node launch file, which will fight any bag-playback or simulation launch.
- Node parameters are set as global `/planner/*` rather than private (`~`) parameters.

### Documentation

- `README.md` has an empty Introduction, no build, dependency or run instructions, and describes a class design (`Cone.toLocal`, `Path.getPosterior`, `getLikelihood`, "Map?") that does not correspond to the code. Most methods it lists as "Not Implemented Yet" never appeared; the ones that were implemented are not listed. The cleaner, the skidpad node, parameters, topics and frames are undocumented.
- `package.xml` says MIT but there is no `LICENSE` file.

### History and process

- 31 commits, 1 pull request. Messages such as `:")`, "minor change", "Minor changes", "Parameter update" carry no information.
- The commit "Improved Robustness" deleted the IPG and AirSim simulator bridges, the plotter node and `add_noise.py`, which were the only tooling for exercising the planner. Nothing replaced them.
- Parameter tuning was done by editing constants in source (`consts.py`), so tuned values are indistinguishable from code changes in history.

## 9. What is good

- The architecture is right for a first version: a node thin enough to be a shell, a pure-Python planner library that can run without ROS (the harness proves it), and a separate post-processing stage.
- The algorithm is a reasonable, well-known approach (Delaunay midpoints, beam search with a cost combining curvature, track width and colour likelihood) and, with the colour convention right, produces a correct centreline on straight and curved synthetic tracks.
- It is fast enough: tens of milliseconds per plan in pure Python.
- Shared state between the subscriber thread and the planning loop is protected by a mutex; there is a status heartbeat; the cleaner has docstrings and a YAML parameter file.

## 10. Recommendations, in priority order

**P0: make it buildable and runnable from the repository alone**

1. Pin or vendor `asurt_msgs` and the ROS 1 `tf_helper` (submodule at a fixed commit, or copy the six helper functions into this package; they are about 80 lines).
2. `package.xml`: add `tf_helper` as a build dependency (or drop it from `find_package`, since nothing is generated from it); add `nav_msgs`, `geometry_msgs`, `tf2_ros`, `asurt_msgs`, `python3-numpy`, `python3-scipy`, `python3-yaml`; remove `roscpp`; fill in maintainer, description, version.
3. Rename the package to `fs_planning_2022` (or similar) and raise `cmake_minimum_required` to at least 3.5.
4. Add the shebang to `skidpad.py`; move `params.yaml` to `config/` and load it via `rosparam` in the launch file, or declare it as `package_data`.
5. Fix the startup `None` (R1) and wrap the loop body in try/except with `rospy.logerr`; add `respawn="true"` to the launch file.
6. Resolve the colour convention (P7) using the constants from `Landmark.msg`, and delete the duplicate constants in `consts.py`.

**P1: correctness**

7. `filter_local`: compute `d = atan2(sin(a-h), cos(a-h))` and compare `abs(d)`.
8. `triangulate`: use `hypot(x-l1.x, y-l1.y)`, or simply half the edge length.
9. Headings: average unit vectors (`atan2(sin1+sin2, cos1+cos2)`) and wrap with `(a + π) % 2π - π`; apply the same wrap in `get_largest_angle_change`.
10. Replace slope/intercept with a 2-D vector projection onto the segment; clamp to [0, 1] for "within segment".
11. Clamp probabilities to `[eps, 1-eps]` before `log`; make `get_cost` return one shape.
12. Stop deep-copying `Path`: share `cones`/`color_probs`, copy only the per-path lists.
13. Cleaner: delete the unreachable block and unused parameters; prune passed waypoints instead of appending indices forever; guard `splprep` against duplicate points; reconsider the 2 m reset.
14. `ArcLengthSpline`: store the knot arclengths and use `np.searchsorted`; clamp the interval; use `CubicSpline.__call__` instead of re-implementing polynomial evaluation.

**P2: engineering hygiene**

15. Add `pytest` tests for the planner on synthetic tracks (straight, left/right arcs, hairpin, one-colour-only, no cones, single cone) and for the cleaner; add a `rostest` that launches the node with a bag or the synthetic world in `review/ros_stubs/fake_world.py`.
16. Add CI (GitHub Actions with a `ros:noetic` container): `catkin_lint`, `flake8`/`pylint`, `pytest`, `catkin_make install`.
17. Add pre-commit with `black`, `isort`, `flake8`, `end-of-file-fixer`, `mixed-line-ending`; normalise file modes; add `*.egg-info` to `.gitignore`.
18. Rewrite `README.md` to describe what exists: topics, frames, parameters, how to build and run, the algorithm; add `LICENSE`.
19. Replace `print` with `rospy.log*`; make constants ROS parameters (or `dynamic_reconfigure`) so tuning is not a source change.
20. Since `tf_helper` has already moved to ROS 2, plan the ROS 2 port of this package rather than restoring the ROS 1 helper long-term.

## 11. Reproduction

Everything used in this review lives in `review/`:

| Path | Purpose |
|---|---|
| `review/harness/harness.py`, `harness_swapped.py` | Drive `Planner` on synthetic tracks with both colour conventions; prints chosen waypoints, cost and timing. Needs only `numpy` and `scipy`. |
| `review/harness/microtests.py` | One reproduction per planner bug in section 6. |
| `review/harness/cleaner_harness.py` | Spline and cleaner checks from section 7 and the headless skidpad geometry check. |
| `review/stubs_pkg/tf_helper/` | Minimal non-ROS stand-in for the names `planner.py` imports. |
| `review/ros_stubs/asurt_msgs/`, `review/ros_stubs/tf_helper/` | Stand-in catkin packages so the workspace configures and builds. Message fields are inferred from usage. |
| `review/ros_stubs/fake_world.py` | Publishes cones, TF tree and records `/planning/path`. Arguments: left type, right type, seconds, output JSON. |
| `review/ros_stubs/run_tests*.sh` | The runtime scenarios of section 5 (set `SCRATCH` to a workspace root containing `ws/`, `ros/`, `mm/`). |
| `review/logs/` | Captured node output for R1 and JSON timelines for R2, R3, R4. |

Environment used: Ubuntu 24.04, RoboStack `ros-noetic-ros-base` via micromamba (Python 3.11, numpy 1.26, CMake 4.4). Two environment-level workarounds were needed that are not the repository's fault: `pip install empy==3.3.4` (message generation) and `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` for catkin's own scripts (the repository needs it too, see B2).

```bash
# pure-Python checks
pip install numpy scipy matplotlib pyyaml
python review/harness/microtests.py
python review/harness/harness_swapped.py
python review/harness/cleaner_harness.py

# catkin build (inside a ROS Noetic shell)
mkdir -p ws/src && cp -r review/ros_stubs/asurt_msgs review/ros_stubs/tf_helper ws/src/ && ln -s "$PWD" ws/src/fs_planning-2022
cd ws && catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCATKIN_WHITELIST_PACKAGES="asurt_msgs;tf_helper"
catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCATKIN_WHITELIST_PACKAGES=""   # second pass needed because of B1
catkin_make install -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

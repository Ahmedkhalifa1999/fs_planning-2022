import sys, math, warnings, time
import os; _H = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(_H, "..", "stubs_pkg")); sys.path.insert(0, os.path.join(_H, "..", "..", "src"))
import matplotlib; matplotlib.use("Agg")
import numpy as np
warnings.simplefilter("always")
from waypoints_cleaner import WaypointsCleaner, ArcLengthSpline
from waypoints_cleaner.spline_interpolator import SplineInterpolator

print("=== A. SplineInterpolator.evaluate outside the knot range")
si = SplineInterpolator(); si.fit(np.array([0.,1.,2.,3.]), np.array([0.,1.,0.,1.]))
for x in [-0.5, 1.5, 2.99, 3.0, 3.5]:
    try: print(f"   x={x}: {si.evaluate(x):.3f}")
    except Exception as e: print(f"   x={x}: {type(e).__name__}: {e}")

print("=== B. ArcLengthSpline accuracy on a circle of radius 5 (60 knots)")
th = np.linspace(0, 2*np.pi, 60)
acs = ArcLengthSpline(num_samples=20, arclength_dt=0.05)
acs.fit_spline(5*np.cos(th), 5*np.sin(th))
print(f"   estimated total arclength {acs.total_arclength:.3f} vs true {2*np.pi*5:.3f}")
print(f"   knot t (arclength) values: {np.round(acs.t_points,2)}")
print(f"   evaluate() assumes knots are multiples of delta={acs.delta_arclength:.3f}; max knot deviation from multiple = {np.max(np.abs(acs.t_points - np.arange(len(acs.t_points))*acs.delta_arclength)):.3f} m")
errs = []
for s in np.linspace(0, acs.total_arclength-0.02, 200):
    x, y = acs.evaluate(s); errs.append(abs(math.hypot(x, y) - 5))
print(f"   radial error of evaluate(): mean {np.mean(errs):.3f} m, max {np.max(errs):.3f} m")
try:
    x, y = acs.evaluate(acs.total_arclength)
    print(f"   evaluate(total_arclength) ok: ({x:.2f},{y:.2f})")
except Exception as e: print(f"   evaluate(total_arclength): {type(e).__name__}: {e}")
pts = acs.get_points(); print(f"   get_points() -> {pts.shape[0]} points")

print("=== C. ArcLengthSpline with duplicate / degenerate input")
for name, xs, ys in [("all identical", np.zeros(5), np.zeros(5)), ("2 points", np.array([0.,1.]), np.array([0.,0.])), ("3 points", np.array([0.,1.,2.]), np.array([0.,1.,0.]))]:
    try:
        a = ArcLengthSpline(num_samples=10, arclength_dt=0.1); a.fit_spline(xs, ys); p = a.get_points(); print(f"   {name}: ok, {p.shape[0]} points")
    except Exception as e: print(f"   {name}: {type(e).__name__}: {e}")

print("=== D. WaypointsCleaner end-to-end, car driving +x at 1 m/tick, planner sees centreline 1.5..4.5 m ahead")
cl = WaypointsCleaner(to_rear=[-1.0, 0.0])
t0 = time.perf_counter()
for tick in range(30):
    pos = np.array([tick*1.0, 0.0])
    cl.update_position(pos, 0.0)
    local = np.array([[1.5, 0.0], [3.0, 0.0], [4.5, 0.0]]) + np.random.default_rng(tick).normal(0, 0.15, (3,2))
    cl.add_waypoints(local)
    out = cl.get_waypoints()
    if tick in (0, 1, 2, 3, 5, 10, 29):
        print(f"   tick {tick:2d}: stored={len(cl.prev_waypoints):3d} done_idx_entries={len(cl.prev_done_waypoints):4d} out={out.shape} first={np.round(out[:1],2).tolist()} last={np.round(out[-1:],2).tolist()}")
print(f"   30 ticks took {(time.perf_counter()-t0)*1000:.1f} ms")
print("   unused params from yaml:", [k for k in ("num_waypoints_return","num_frames_use","theta_lookahead","plot") if hasattr(cl,k)])

print("=== E. WaypointsCleaner when the car pose jumps (map->odom correction) or to_rear is zero")
cl2 = WaypointsCleaner(to_rear=[0.0, 0.0])
cl2.update_position([100.0, 50.0], 0.0)   # first pose far from origin -> reset() is called immediately
cl2.add_waypoints(np.array([[1.5,0],[3,0],[4.5,0]]))
cl2.update_position([100.5, 50.0], 0.0); cl2.add_waypoints(np.array([[1.5,0],[3,0],[4.5,0]]))
cl2.update_position([101.0, 50.0], 0.0); cl2.add_waypoints(np.array([[1.5,0],[3,0],[4.5,0]]))
try: print("   to_rear=0 ->", cl2.get_waypoints().shape)
except Exception as e: print("   to_rear=0 ->", type(e).__name__, e)
cl2.update_position([104.0, 50.0], 0.0)
print("   after a 3 m jump: stored waypoints =", len(cl2.prev_waypoints))

print("=== F. skidpad.py geometry with rospy stubbed (headless)")
import types, importlib.util
rospy = types.ModuleType("rospy"); rospy.init_node = lambda *a, **k: None; rospy.spin = lambda: None
class Path:
    def __init__(self): self.poses = []; self.header = types.SimpleNamespace(frame_id="")
class Pub:
    def __init__(self, *a, **k): pass
    def publish(self, m): print("   published Path with", len(m.poses), "poses in frame", m.header.frame_id)
rospy.Publisher = Pub; sys.modules["rospy"] = rospy
navm = types.ModuleType("nav_msgs.msg"); navm.Path = Path
sys.modules["nav_msgs"] = types.ModuleType("nav_msgs"); sys.modules["nav_msgs.msg"] = navm
tfh = types.ModuleType("tf_helper")
def create_path_message(wps, frame):
    p = Path(); p.header.frame_id = frame; p.poses = [tuple(w) for w in wps]; return p
tfh.create_path_message = create_path_message; sys.modules["tf_helper"] = tfh
import matplotlib.pyplot as plt; n_pause = [0]
plt.pause = lambda *a: n_pause.__setitem__(0, n_pause[0] + 1)
t0 = time.perf_counter()
spec = importlib.util.spec_from_file_location("skidpad", os.path.join(_H, "..", "..", "src", "nodes", "skidpad.py"))
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
print(f"   {m.points2.shape[0]} path points; plt.pause called {n_pause[0]} times (0.01 s each = {n_pause[0]*0.01:.1f} s of GUI animation before publishing)")
d = np.linalg.norm(np.diff(m.points2, axis=0), axis=1)
print(f"   spacing between consecutive points: min {d.min():.3f} max {d.max():.3f} m (arclength_dt=0.1 requested)")

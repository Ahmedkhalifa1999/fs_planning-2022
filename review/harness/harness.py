import sys, os, time, math
_H = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_H, "..", "stubs_pkg"))
sys.path.insert(0, os.path.join(_H, "..", "..", "src"))
import numpy as np
from planner import Planner
from planner.map import Map
from planner.planner import Landmark

class P:  # fake geometry_msgs Point
    def __init__(self, x, y): self.x, self.y = x, y
class LM:  # fake asurt_msgs Landmark
    def __init__(self, x, y, t): self.position, self.type = P(x, y), t
class LMA:
    def __init__(self, lms): self.landmarks = lms

def straight_track(n=8, spacing=2.5, width=3.0, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    lms = []
    for i in range(n):
        x = 1.0 + i*spacing
        lms.append(LM(x + rng.normal(0, noise),  width/2 + rng.normal(0, noise), 0))  # type 0 -> blue (left)
        lms.append(LM(x + rng.normal(0, noise), -width/2 + rng.normal(0, noise), 1))  # type 1 -> yellow (right)
    return lms

def arc_track(n=10, spacing=2.5, width=3.0, radius=8.0, seed=0):
    lms = []
    for i in range(n):
        s = 1.0 + i*spacing
        th = s/radius
        cx, cy = radius*math.sin(th), radius*(1-math.cos(th))  # turning left
        nx, ny = -math.sin(th), math.cos(th)  # left normal
        lms.append(LM(cx + nx*width/2, cy + ny*width/2, 0))
        lms.append(LM(cx - nx*width/2, cy - ny*width/2, 1))
    return lms

def run(name, lms, reps=3):
    pl = Planner("base_link")
    pl.landmark_cb(LMA(lms))
    t0 = time.perf_counter(); out = pl.run(); dt = time.perf_counter()-t0
    if out is None:
        print(f"[{name}] planner returned None ({dt*1000:.1f} ms)"); return
    best, possible = out
    print(f"[{name}] {len(lms)} cones -> {possible.shape[0]} candidate waypoints, {dt*1000:.1f} ms")
    if best is None:
        print("   best_path is None"); return
    wp = np.array([[w.x, w.y] for w in best.waypoints])
    cost = best.get_cost()
    print("   best path waypoints:", np.round(wp, 2).tolist())
    print("   cost tuple len", len(cost), "cost=", np.round(cost[0], 3))
    # timing over reps
    ts = []
    for _ in range(reps):
        pl.landmark_cb(LMA(lms)); t0 = time.perf_counter(); pl.run(); ts.append(time.perf_counter()-t0)
    print(f"   timing over {reps} runs: mean {np.mean(ts)*1000:.1f} ms, max {np.max(ts)*1000:.1f} ms")

run("straight 8 pairs", straight_track(8))
run("straight 8 pairs noisy", straight_track(8, noise=0.2, seed=1))
run("straight 20 pairs (only <10m used)", straight_track(20))
run("left arc r=8", arc_track(10))
run("2 cones only", straight_track(1))
run("3 collinear cones", [LM(2,1.5,0), LM(4,1.5,0), LM(6,1.5,0)])
run("1 cone", [LM(2,1.5,0)])
run("0 cones", [])
run("unknown colors", [LM(x, 1.5, 4) for x in range(1,9)] + [LM(x, -1.5, 4) for x in range(1,9)])
run("dense 50 pairs within 10m", straight_track(50, spacing=0.2))

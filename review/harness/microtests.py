import sys, math, warnings
import os; _H = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(_H, "..", "stubs_pkg")); sys.path.insert(0, os.path.join(_H, "..", "..", "src"))
import numpy as np
warnings.simplefilter("always")
from planner.map import Map
from planner.waypoint import Waypoint
from planner.path import Path
from planner.planner import Landmark
from planner.consts import *

print("=== 1. Map.filter_local field-of-view filter (FOV=180 deg, car at origin heading 0)")
for ang in [0, 45, 90, 135, 180, -45, -90, -135]:
    lm = Landmark(2*math.cos(math.radians(ang)), 2*math.sin(math.radians(ang)), 4)
    kept = len(Map.filter_local([lm], 180, 10)) == 1
    print(f"   cone at {ang:5d} deg -> {'KEPT' if kept else 'dropped'}")

print("=== 2. Map.triangulate half-edge-length check uses x twice")
l1, l2 = Landmark(0, 0, 4), Landmark(0, 6, 4)   # vertical edge, 6 m long -> midpoint is 3 m from each cone
x, y = (l1.x+l2.x)/2, (l1.y+l2.y)/2
buggy = np.sqrt((x-l1.x)**2 + (x-l1.y)**2)
true  = np.sqrt((x-l1.x)**2 + (y-l1.y)**2)
print(f"   code computes dist={buggy:.2f}, real half-length={true:.2f}, threshold TRACKWIDTH/2+2={TRACKWIDTH/2+2}")
l1, l2 = Landmark(0, 0, 4), Landmark(6, 0, 4)   # horizontal edge, same length
x, y = (l1.x+l2.x)/2, (l1.y+l2.y)/2
buggy = np.sqrt((x-l1.x)**2 + (x-l1.y)**2)
print(f"   horizontal 6 m edge: code computes dist={buggy:.2f} (accepted={buggy < TRACKWIDTH/2+2}), vertical 6 m edge accepted={np.sqrt((0-0)**2+(0-0)**2) < TRACKWIDTH/2+2}")

print("=== 3. Waypoint.update_heading_next angle averaging near +/-pi")
w = Waypoint(0, 0); w.heading = math.radians(170)
w.update_heading_next(Waypoint(math.cos(math.radians(-170)), math.sin(math.radians(-170))))
print(f"   avg of 170 deg and -170 deg -> {math.degrees(w.heading):.1f} deg (geometrically should be 180)")
w = Waypoint(0, 0); w.heading = math.radians(179)
w.update_heading_next(Waypoint(math.cos(math.radians(179)), math.sin(math.radians(179))))
print(f"   avg of 179 and 179 -> {math.degrees(w.heading):.1f}")
w = Waypoint(0,0); w.heading = 3.5  # out of range input to see the wrap
w.update_heading_next(Waypoint(-1, 0.01))
print(f"   heading 3.5 rad then next at ~180deg -> {w.heading:.3f} rad (wrap subtracts pi, not 2*pi)")

print("=== 4. Path.get_slope_and_intercept when x decreases (denom negative)")
p = Path(np.zeros((1,2)), np.zeros((1,3)))
with warnings.catch_warnings(record=True) as wl:
    s, b, n, bound = p.get_slope_and_intercept(Waypoint(1, 0), Waypoint(0, 1))   # dx = -1
    print(f"   dx=-1, dy=+1: slope={s:.4f} (true -1.0)")
    try:
        s, b, n, bound = p.get_slope_and_intercept(Waypoint(0, 0), Waypoint(-1e-4, 1)); print("   dx=-1e-4 ok")
    except Exception as e: print(f"   dx=-1e-4 -> {type(e).__name__}: {e}  (would kill the node's main loop)")
    s, b, n, bound = p.get_slope_and_intercept(Waypoint(0, 0), Waypoint(0, 1))
    print(f"   vertical segment: slope={s:.0f}")

print("=== 5. Path.get_largest_angle_change has no wrap-around")
p = Path(np.array([[10.,10.]]), np.array([[.33,.33,.33]]), [Waypoint(0,0)])
p.add_waypoint(-1, 0.01)    # heading ~ +179.4 deg
p.add_waypoint(-2, 0.0)     # heading ~ -179.4 deg
print(f"   two nearly-straight segments pointing backwards -> largest change = {math.degrees(p.get_largest_angle_change()):.1f} deg, cost[0]={p.get_cost()[0]:.1f}")

print("=== 6. Path.color_cost with a probability of exactly 1.0")
p = Path(np.array([[1.,1.5]]), np.array([[1.0,0.0,0.0]]), [Waypoint(0,0)])
with warnings.catch_warnings(record=True) as wl:
    p.add_waypoint(1, 0); p.add_waypoint(2, 0)
    c = p.get_cost()
    print(f"   cost = {c[0]}  warnings={[w.category.__name__ for w in wl]}")

print("=== 7. Cone class attribute sharing")
from planner.cone import Cone
a, b = Cone(), Cone()
a.color_probs.append("oops")
print(f"   b.color_probs after mutating a's = {b.color_probs}")

print("=== 8. get_cost tuple length differs by branch")
p = Path(np.array([[1.,1.5]]), np.array([[.5,.3,.2]]), [Waypoint(0,0)])
p.add_waypoint(1, 0); p.add_waypoint(0, 0.5)   # sharp turn -> angle>2.5 branch
print("   sharp-turn branch returns", len(p.get_cost()), "values; normal branch returns 7")
try:
    p.get_cost()[3:5]; print("   verbose slice [3:5] on sharp branch ->", p.get_cost()[3:5])
except Exception as e: print("   ", type(e).__name__, e)

print("=== done. Planner with color convention swapped (type 1 = left/blue in this code)")


import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from tf_helper import *
from waypoints_cleaner.spline_interpolator import ArcLengthSpline
rospy.init_node("skidpad_pub")

center_front = 5
center_dist_perp = 4.5
radius = 5
dist_front_to_start = 9

def get_y_intersections(x,y,r):
    s = r*r - x*x
    sqrt = np.sqrt(s)
    angle = np.arcsin(sqrt/r)
    return y+sqrt, y-sqrt, angle

def circle_samples(x,y,r, arcs):
    circle = [r*np.cos(arcs),r*np.sin(arcs)]
    circle = np.array(circle).transpose()
    circle[:,0] += x
    circle[:,1] += y
    return circle

y_ints = get_y_intersections(center_dist_perp, center_front, radius)
points = [[0,0],[0,y_ints[0]]]
arcs_right = np.arange(np.pi-y_ints[2], -np.pi+y_ints[2] , -0.1)
points.extend(circle_samples(center_dist_perp, center_front, radius, arcs_right))
points.extend([[0,y_ints[0]]])
arcs_left = np.arange(y_ints[2], 2*np.pi-y_ints[2], 0.1)
points.extend(circle_samples(-center_dist_perp, center_front, radius, arcs_left))
points.extend([[0,dist_front_to_start]])

points = np.array(points)


# Smooth points
ac_spline = ArcLengthSpline(num_samples=20,arclength_dt=0.1)
ac_spline.fit_spline(points[:,0],points[:,1])
points = ac_spline.get_points()

for i in range(points.shape[0]):
    plt.plot(points[:i,0], points[:i,1])
    plt.pause(0.01)

points2 = np.copy(points)
points2[:,0] = points[:,1]
points2[:,1] = -1*points[:,0]
path_pub = rospy.Publisher("planning/path", Path, queue_size=1,latch=True)
skidpad_path = create_path_message(points2, "map")
path_pub.publish(skidpad_path)
rospy.spin()





#!/usr/bin/env python
"""Publishes a synthetic straight track on /SLAM/cones (in base_link) at 10 Hz, broadcasts the TF tree the node needs
(map->base_link moving at 1 m/s, base_link->rear_link, base_link->kit), and records /planning/path + /status/planning."""
import sys, math, time, json
import rospy, tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from asurt_msgs.msg import Landmark, LandmarkArray

LEFT_TYPE  = int(sys.argv[1]) if len(sys.argv) > 1 else 0    # cone type placed on the +y (left) side
RIGHT_TYPE = int(sys.argv[2]) if len(sys.argv) > 2 else 1
DURATION   = float(sys.argv[3]) if len(sys.argv) > 3 else 12.0
OUT        = sys.argv[4] if len(sys.argv) > 4 else "/tmp/fake_world_out.json"

rospy.init_node("fake_world"); t0 = time.time()
br = tf2_ros.TransformBroadcaster()
sbr = tf2_ros.StaticTransformBroadcaster()
def tfmsg(parent, child, x, y, yaw=0.0):
    t = TransformStamped(); t.header.stamp = rospy.Time.now(); t.header.frame_id = parent; t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y = x, y
    t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
    return t
sbr.sendTransform([tfmsg("base_link", "rear_link", -1.0, 0.0), tfmsg("base_link", "kit", 0.5, 0.0)])

cone_pub = rospy.Publisher("/SLAM/cones", LandmarkArray, queue_size=1)
paths, statuses = [], []
def on_path(m): paths.append({"t": round(time.time()-t0,2), "frame": m.header.frame_id, "n": len(m.poses), "pts": [[round(p.pose.position.x,2), round(p.pose.position.y,2)] for p in m.poses[:6]]})
def on_status(m): statuses.append(m.data)
rospy.Subscriber("/planning/path", Path, on_path); rospy.Subscriber("/status/planning", String, on_status)
rospy.Subscriber("/planner/possible_waypoints", LandmarkArray, lambda m: paths.append({"possible": len(m.landmarks)}) if len(paths) < 3 else None)

rate = rospy.Rate(10); i = 0
while not rospy.is_shutdown() and time.time() - t0 < DURATION:
    car_x = (time.time() - t0) * 1.0                      # car drives +x at 1 m/s in map
    br.sendTransform(tfmsg("map", "base_link", car_x, 0.0, 0.0))
    msg = LandmarkArray(); msg.header.stamp = rospy.Time.now(); msg.header.frame_id = "base_link"
    for k in range(-2, 12):                               # cone pairs every 2.5 m, 3 m track width, in base_link
        cx = 1.0 + k*2.5 - (car_x % 2.5)
        for y, t in ((1.5, LEFT_TYPE), (-1.5, RIGHT_TYPE)):
            lm = Landmark(); lm.position.x, lm.position.y, lm.type = cx, y, t; msg.landmarks.append(lm)
    cone_pub.publish(msg); i += 1; rate.sleep()
json.dump({"cone_msgs_sent": i, "paths_received": len([p for p in paths if "n" in p]), "statuses": statuses, "samples": paths[:3] + paths[-2:], "timeline": [(p["t"], p["n"]) for p in paths if "n" in p], "first_nonempty": next((p for p in paths if p.get("n",0) > 0), None)}, open(OUT, "w"), indent=1)

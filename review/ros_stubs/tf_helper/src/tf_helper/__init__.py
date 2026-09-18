"""Stand-in for the ROS1 tf_helper API (upstream has since moved to ROS2 and dropped these names).
Only the names fs_planning-2022 pulls in via `from tf_helper import *` are provided."""
from functools import wraps
import numpy as np
import rospy
import tf2_ros
from std_msgs.msg import String, Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from asurt_msgs.msg import Landmark, LandmarkArray

__all__ = ["np", "mutex_lock", "TF_Helper", "Status_Publisher", "create_path_message", "create_landmark_message"]

def mutex_lock(lock):
    def deco(fn):
        @wraps(fn)
        def wrapper(*a, **k):
            with lock:
                return fn(*a, **k)
        return wrapper
    return deco

def _yaw(q):
    return np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

class TF_Helper:
    def __init__(self, name):
        self.name = name
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

    def _lookup(self, target, source, timeout=2.0):
        try:
            return self.buffer.lookup_transform(target, source, rospy.Time(0), rospy.Duration(timeout))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(2.0, "[tf_helper stub] %s -> %s unavailable: %s", source, target, e)
            return None

    def get_transform(self, child_frame, parent_frame):
        """Pose of child_frame expressed in parent_frame -> ((x, y), yaw) or None."""
        t = self._lookup(parent_frame, child_frame)
        if t is None:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return (np.array([tr.x, tr.y]), float(_yaw(q)))

    def get_message_in(self, msg, frame):
        src = msg.header.frame_id
        if src == frame or src == "":
            msg.header.frame_id = frame
            return msg
        t = self._lookup(frame, src)
        if t is None:
            return None
        tr, yaw = t.transform.translation, _yaw(t.transform.rotation)
        c, s = np.cos(yaw), np.sin(yaw)
        def tf_xy(x, y):
            return c*x - s*y + tr.x, s*x + c*y + tr.y
        if isinstance(msg, LandmarkArray):
            for lm in msg.landmarks:
                lm.position.x, lm.position.y = tf_xy(lm.position.x, lm.position.y)
        elif isinstance(msg, Path):
            for p in msg.poses:
                p.pose.position.x, p.pose.position.y = tf_xy(p.pose.position.x, p.pose.position.y)
                p.header.frame_id = frame
        msg.header.frame_id = frame
        return msg

class Status_Publisher:
    def __init__(self, topic):
        self.pub = rospy.Publisher(topic, String, queue_size=1, latch=True)
    def _send(self, s): self.pub.publish(String(data=s))
    def starting(self): self._send("starting")
    def ready(self): self._send("ready")
    def running(self): self._send("running")

def create_path_message(waypoints, frame_id):
    path = Path(); path.header = Header(frame_id=frame_id, stamp=rospy.Time.now())
    for w in np.asarray(waypoints).reshape(-1, 2):
        path.poses.append(PoseStamped(Header(frame_id=frame_id), Pose(Point(float(w[0]), float(w[1]), 0.0), Quaternion(0, 0, 0, 1))))
    return path

def create_landmark_message(cones, types, frame_id):
    msg = LandmarkArray(); msg.header = Header(frame_id=frame_id, stamp=rospy.Time.now())
    for c, t in zip(np.asarray(cones).reshape(-1, 2), types):
        lm = Landmark(); lm.position.x, lm.position.y, lm.type = float(c[0]), float(c[1]), int(t)
        msg.landmarks.append(lm)
    return msg

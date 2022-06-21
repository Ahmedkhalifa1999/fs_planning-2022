#!/usr/bin/python3
import tf
import time
import rospy
import tf2_ros

import numpy as np

from asurt_msgs.msg import LandmarkArray
from nav_msgs.msg import Path as navPath
from nav_msgs.msg import Odometry
from planner import *
from waypoints_cleaner import WaypointsCleaner
from tf_helper import *


def perception_callback(landmark_array):
    global planner_py, tf_help
    landmarks = tf_help.get_message_in(landmark_array, "velodyne")
    planner_py.landmark_cb(landmarks)


def main():
    global planner_py, tf_help
    rospy.init_node('path_planner')
    status = Status_Publisher('/status/planning')
    status.starting()
    
    planner_py = Planner()

    MAP_TOPIC = rospy.get_param("planner/map_topic")
    WAYPOINTS_TOPIC = rospy.get_param("planner/waypoints_topic")
    tf_help = TF_Helper("planning")

    rospy.Subscriber(MAP_TOPIC, LandmarkArray, perception_callback)
    waypoints_pub = rospy.Publisher(WAYPOINTS_TOPIC, navPath, queue_size=1)
    
    to_rear = tf_help.get_transform("rear_link", "velodyne")
    cleaner = WaypointsCleaner(to_rear[0])
    status.ready()

    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        rate.sleep()

        pose = tf_help.get_transform("velodyne", "map")
        if pose is None:
            continue

        best_path = planner_py.run()
        cleaner.update_position(*pose)

        if best_path is None: # No path found
            continue

        waypoints = np.array([[w.x,w.y] for w in best_path.waypoints])
        cleaner.add_waypoints(waypoints[1:])
        new_waypoints = cleaner.get_waypoints()
        
        # Create and publish message
        output_path = create_path_message(new_waypoints, "map")
        output_path = tf_help.get_message_in(output_path, "rear_link")
        waypoints_pub.publish(output_path)
        status.running()

if __name__ == '__main__':
    main()

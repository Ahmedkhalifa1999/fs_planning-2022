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

landmark_array = None
planner_py = Planner()
def perception_callback(data):
    global landmark_array, planner_py, tf_help
    landmark_array = data
    planning_frame_id = "velodyne"
    landmarks = tf_help.get_message_in(landmark_array, planning_frame_id)
    planner_py.landmark_cb(landmarks)


def main():
    global tf_help
    rospy.init_node('path_planner')

    MAP_TOPIC = rospy.get_param("planner/map_topic")
    WAYPOINTS_TOPIC = rospy.get_param("planner/waypoints_topic")
    planning_frame_id = "velodyne"
    output_frame_id = "base_link"
    tf_help = TF_Helper("planning")

    rospy.Subscriber(MAP_TOPIC, LandmarkArray, perception_callback)
    waypoints_pub = rospy.Publisher(WAYPOINTS_TOPIC, navPath, queue_size=1)
    cleaner = WaypointsCleaner()


    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        rate.sleep()

        pose = tf_help.get_transform("velodyne", "map")
        if landmark_array is None or pose is None:
            continue

        best_path = planner_py.run()
        cleaner.update_position(*pose)

        if best_path is None:
            rospy.loginfo("No Path Found")
            continue

        waypoints = np.array([[w.x,w.y] for w in best_path.waypoints[1:]])
        cleaner.add_waypoints(waypoints[1:])
        new_waypoints = cleaner.get_waypoints()
        
        # Create and publish message
        output_path = create_path_message(new_waypoints, "map")
        #output_path = create_path_message(waypoints, "velodyne")
        output_path = tf_help.get_message_in(output_path, output_frame_id)
        #output_path = add_start_waypoint(output_path)
        waypoints_pub.publish(output_path)

if __name__ == '__main__':
    main()

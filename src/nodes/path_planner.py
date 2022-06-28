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
    landmarks = tf_help.get_message_in(landmark_array, planner_py.planning_frame)
    planner_py.landmark_cb(landmarks)


def main():
    global planner_py, tf_help
    rospy.init_node('path_planner')
    status = Status_Publisher('/status/planning')
    status.starting()
    
    planner_py = Planner("base_link")

    MAP_TOPIC = rospy.get_param("planner/map_topic")
    WAYPOINTS_TOPIC = rospy.get_param("planner/waypoints_topic")
    possible_waypoints_topic = "planner/possible_waypoints"
    use_cleaner = rospy.get_param("planner/use_cleaner")
    tf_help = TF_Helper("planning")

    rospy.Subscriber(MAP_TOPIC, LandmarkArray, perception_callback)
    waypoints_pub = rospy.Publisher(WAYPOINTS_TOPIC, navPath, queue_size=1)
    possible_waypoints_pub = rospy.Publisher(possible_waypoints_topic, LandmarkArray, queue_size=1)
    
    to_rear = tf_help.get_transform("rear_link", planner_py.planning_frame)
    cleaner = WaypointsCleaner(to_rear[0])
    status.ready()

    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        rate.sleep()

        planner_output = planner_py.run()

        if planner_output is None: # No path found
            continue
        best_path, possible_waypoints = planner_output
        if best_path is None:
            continue

        waypoints = np.array([[w.x,w.y] for w in best_path.waypoints])

        pose = tf_help.get_transform(planner_py.planning_frame, "map")
        if use_cleaner and not pose is None:
            cleaner.update_position(*pose)
            cleaner.add_waypoints(waypoints[1:])
            waypoints = cleaner.get_waypoints()
            output_path = create_path_message(waypoints, "map")
        else:
            output_path = create_path_message(waypoints, planner_py.planning_frame)

        # Publish message
        output_path = tf_help.get_message_in(output_path, "kit")
        waypoints_pub.publish(output_path)

        possible_waypoints_pub.publish(create_landmark_message(possible_waypoints, np.ones(possible_waypoints.shape[0])*4, planner_py.planning_frame))
        status.running()

if __name__ == '__main__':
    main()

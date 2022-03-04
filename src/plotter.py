#! /usr/bin/env python3
import rospy
import matplotlib.pyplot as plt

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as navPath
from asurt_msgs.msg import LandmarkArray, Landmark

MAP_TOPIC = rospy.get_param("planner/map_topic", "/planner/cones")
POSE_TOPIC = rospy.get_param("planner/pose_topic", "/planner/pose")
WAYPOINTS_TOPIC = rospy.get_param("planner/waypoints_topic", "/planner/path")

BLUE_CONE_STYLE = rospy.get_param("planner/blue_cone_style", 0)
YELLOW_CONE_STYLE = rospy.get_param("planner/yellow_cone_style", 1)
ORANGE_CONE_STYLE = rospy.get_param("planner/orange_cone_style", 2)
BIG_CONE_STYLE = rospy.get_param("planner/big_cone_style", 3)
UNKNOWN_CONE_STYLE = rospy.get_param("planner/unknown_cone_style", 4)


rospy.init_node('path_planning_plotting')

while not rospy.is_shutdown():
    
    car_pose: Odometry = rospy.wait_for_message(POSE_TOPIC, Odometry)
    landmarks: LandmarkArray = rospy.wait_for_message(MAP_TOPIC, LandmarkArray)
    path: navPath = rospy.wait_for_message(WAYPOINTS_TOPIC, navPath)

    current_global_right_cones_x = []
    current_global_right_cones_y = []
    current_global_left_cones_x = []
    current_global_left_cones_y = []
    landmark: Landmark
    for landmark in landmarks.landmarks:
        if (landmark.type == YELLOW_CONE_STYLE):
            current_global_right_cones_x.append(landmark.position.x)
            current_global_right_cones_y.append(landmark.position.y)
        elif (landmark.type == BLUE_CONE_STYLE):
            current_global_left_cones_x.append(landmark.position.x)
            current_global_left_cones_y.append(landmark.position.y)
    plt.plot(current_global_right_cones_x, current_global_right_cones_y, 'o', color='yellow')
    plt.plot(current_global_left_cones_x, current_global_left_cones_y, 'o', color='blue')

    waypoint: PoseStamped
    for waypoint in path.poses:
        plt.plot(waypoint.pose.position.x, waypoint.pose.position.y, 'x')

    plt.plot(car_pose.pose.pose.position.x, car_pose.pose.pose.position.y, 'D')
    plt.pause(0.001)
    plt.cla()

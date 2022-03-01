#! /usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
import time

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path as navPath
from asurt_msgs.msg import LandmarkArray, Landmark

MEDIATOR_MAP_TOPIC = rospy.get_param("planner/mediator_map_topic", "airsim/Cones")
MEDIATOR_POSE_TOPIC = rospy.get_param("planner/mediator_pose_topic", "airsim/KinematicOdometry")
MAP_TOPIC = rospy.get_param("planner/map_topic", "/planner/cones")
POSE_TOPIC = rospy.get_param("planner/pose_topic", "/planner/pose")
WAYPOINTS_TOPIC = rospy.get_param("planner/waypoints_topic", "/planner/path")
ADD_NOISE = rospy.get_param("planner/add_noise", False)
PLOTTING = rospy.get_param("planner/plotting", False)

BLUE_CONE_STYLE = rospy.get_param("planner/blue_cone_style", 0)
YELLOW_CONE_STYLE = rospy.get_param("planner/yellow_cone_style", 1)
ORANGE_CONE_STYLE = rospy.get_param("planner/orange_cone_style", 2)
BIG_CONE_STYLE = rospy.get_param("planner/big_cone_style", 3)
UNKNOWN_CONE_STYLE = rospy.get_param("planner/unknown_cone_style", 4)


rospy.init_node('path_planning_airsim_mediator')

cones_pub = rospy.Publisher(MAP_TOPIC, LandmarkArray, queue_size=1)
pose_pub = rospy.Publisher(POSE_TOPIC, Odometry, queue_size=1)

while not rospy.is_shutdown():
    
    airsim_car_pose: Odometry = rospy.wait_for_message(MEDIATOR_POSE_TOPIC, Odometry)
    airsim_landmarks: LandmarkArray = rospy.wait_for_message(MEDIATOR_MAP_TOPIC, LandmarkArray)
    
    car_pose = airsim_car_pose
    landmarks = airsim_landmarks

    pose_pub.publish(car_pose)
    cones_pub.publish(landmarks)

    if PLOTTING:
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

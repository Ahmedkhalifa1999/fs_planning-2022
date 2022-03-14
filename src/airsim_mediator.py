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

pose_pub = rospy.Publisher(POSE_TOPIC, Odometry, queue_size = 1)
cones_pub = rospy.Publisher(MAP_TOPIC, LandmarkArray, queue_size = 1)

def publish_odometry(airsim_car_pose):
    car_pose = airsim_car_pose
    pose_pub.publish(car_pose)

def publish_landmarks(airsim_landmarks):
    landmarks = airsim_landmarks
    cones_pub.publish(landmarks)

pose_sub = rospy.Subscriber(MEDIATOR_POSE_TOPIC, Odometry, publish_odometry)
cones_sub = rospy.Subscriber(MEDIATOR_MAP_TOPIC, LandmarkArray, publish_landmarks)

rospy.spin()


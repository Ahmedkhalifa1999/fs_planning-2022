#! /usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path as navPath
from asurt_msgs.msg import LandmarkArray, Landmark
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Imu

MEDIATOR_MAP_TOPIC = rospy.get_param("planner/mediator_map_topic", "/ObjectList")
MEDIATOR_POSE_TOPIC = rospy.get_param("planner/mediator_pose_topic", "/IMU")
MAP_TOPIC = rospy.get_param("planner/map_topic", "planner/cones")
POSE_TOPIC = rospy.get_param("planner/pose_topic", "planner/pose")
WAYPOINTS_TOPIC = rospy.get_param("planner/waypoints_topic", "planner/path")
ADD_NOISE = rospy.get_param("planner/add_noise", False)
PLOTTING = rospy.get_param("planner/plotting", False)

BLUE_CONE_STYLE = rospy.get_param("planner/blue_cone_style", 0)
YELLOW_CONE_STYLE = rospy.get_param("planner/yellow_cone_style", 1)
ORANGE_CONE_STYLE = rospy.get_param("planner/orange_cone_style", 2)
BIG_CONE_STYLE = rospy.get_param("planner/big_cone_style", 3)
UNKNOWN_CONE_STYLE = rospy.get_param("planner/unknown_cone_style", 4)


rospy.init_node('path_planning_ipg_mediator')

cones_pub = rospy.Publisher(MAP_TOPIC, LandmarkArray, queue_size=1)
pose_pub = rospy.Publisher(POSE_TOPIC, Odometry, queue_size=1)

while not rospy.is_shutdown():
    
    ipg_car_pose: Imu = rospy.wait_for_message(MEDIATOR_POSE_TOPIC, Imu)
    ipg_markers: MarkerArray = rospy.wait_for_message(MEDIATOR_MAP_TOPIC, MarkerArray)
    
    car_pose = Odometry()
    #Logical
    car_pose.pose.pose.position.x = ipg_car_pose.orientation.x
    car_pose.pose.pose.position.y = ipg_car_pose.orientation.y
    car_pose.pose.pose.position.z = ipg_car_pose.orientation.z
    #What Actually Works
    car_pose.pose.pose.position.x = 0.0
    car_pose.pose.pose.position.y = 0.0
    car_pose.pose.pose.position.z = 0.0

    landmarks = []
    marker: Marker
    for marker in ipg_markers.markers:
        color: int = UNKNOWN_CONE_STYLE
        if (marker.color.r > 0.9 and marker.color.g > 0.9 and marker.color.b < 0.1):
            color = YELLOW_CONE_STYLE
        elif (marker.color.r < 0.1 and marker.color.g > 0.2 and marker.color.g < 0.4 and marker.color.b > 0.9):
            color = BLUE_CONE_STYLE
        landmarks.append(Landmark(position = Point(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z),
                         type = color, identifier = None))
    landmarks = LandmarkArray(landmarks = landmarks)

    cones_pub.publish(landmarks)
    pose_pub.publish(car_pose)

    path: navPath = rospy.wait_for_message(WAYPOINTS_TOPIC, navPath)

    if PLOTTING:
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

        wayopoint: PoseStamped
        for waypoint in path.poses:
            plt.plot(waypoint.pose.position.x, waypoint.pose.position.y, 'x')

        plt.plot(car_pose.pose.pose.position.x, car_pose.pose.pose.position.y, 'D')
        plt.pause(0.001)
        plt.cla()

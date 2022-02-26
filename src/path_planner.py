#! /usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry, Path as navPath
from asurt_msgs.msg import LandmarkArray, Landmark

from map import Map
from waypoint import Waypoint
from path import Path

MAP_TOPIC = rospy.get_param("/planner/map_topic", "airsim/Cones")
POSE_TOPIC = rospy.get_param("/planner/pose_topic", "fast_slam2/GlobalPose")
WAYPOINTS_TOPIC = rospy.get_param("/planner/waypoints_topic", "waypoints_topic")
REAL_TIME_GLOBAL_PLOTTING = rospy.get_param("/planner/real_time_global_plotting", True)
MAX_DISTANCE_OF_VIEW = rospy.get_param("/planner/max_distance_of_view", 15)

rospy.init_node('path_planner')
waypoints_pub = rospy.Publisher(WAYPOINTS_TOPIC, navPath, queue_size=1)


"""For Testing"""
YELLOW_CONE_STYLE = rospy.get_param("/planner/yellow_cone_style", 1)
BLUE_CONE_STYLE = rospy.get_param("/planner/blue_cone_style", 0)
ORANGE_CONE_STYLE = rospy.get_param("/planner/orange_cone_style", 2)
BIG_CONE_STYLE = rospy.get_param("/planner/big_cone_style", 3)
UNKNOWN_CONE_STYLE = rospy.get_param("/planner/unknown_cone_style", 4)


while not rospy.is_shutdown():
    
    car_pose: Odometry = rospy.wait_for_message(POSE_TOPIC, Odometry)
    landmark_array: LandmarkArray = rospy.wait_for_message(MAP_TOPIC, LandmarkArray)

    map_object = Map(landmark_array.landmarks, car_pose.pose.pose) #map object, totally unrelated to python's map function

    best_path: Path = map_object.get_path()
    waypoint: Waypoint
    if (best_path == None):
        print("No Path Found")
    else:
        plot_path = list(map(lambda R: (R.x, R.y), best_path.waypoints))
        output_path = navPath()
        output_path.header.frame_id = 'map'
        waypoint: Waypoint
        output_path.poses = [PoseStamped(Header(frame_id = 'map'), Pose(Point(x = waypoint.x, y = waypoint.y), Quaternion())) for waypoint in best_path.waypoints]
        waypoints_pub.publish(output_path)

    current_global_right_cones_x = []
    current_global_right_cones_y = []
    current_global_left_cones_x = []
    current_global_left_cones_y = []
    landmark: Landmark
    for landmark in landmark_array.landmarks:
        if (landmark.type == YELLOW_CONE_STYLE):
            current_global_right_cones_x.append(landmark.position.x)
            current_global_right_cones_y.append(landmark.position.y)
        elif (landmark.type == BLUE_CONE_STYLE):
            current_global_left_cones_x.append(landmark.position.x)
            current_global_left_cones_y.append(landmark.position.y)

    if REAL_TIME_GLOBAL_PLOTTING:
        plt.plot(current_global_right_cones_x, current_global_right_cones_y, 'o', color='yellow')
        plt.plot(current_global_left_cones_x, current_global_left_cones_y, 'o', color='blue')

        if (best_path != None): 
            plt.plot(np.array(plot_path)[:,0], np.array(plot_path)[:,1], 'x')    #Goal Midpoint
        plt.plot(car_pose.pose.pose.position.x, car_pose.pose.pose.position.y, 'D')        #Current Pose
        plt.pause(0.001)
        plt.cla()

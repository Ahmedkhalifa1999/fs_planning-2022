#! /usr/bin/env python
from map import Map
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension

from asurt_msgs.msg import LandmarkArray
from asurt_msgs.msg import Landmark
from nav_msgs.msg import Odometry
from waypoint import Waypoint

import time

MAP_TOPIC = rospy.get_param("/planner/map_topic", "airsim/Cones")
POSE_TOPIC = rospy.get_param("/planner/pose_topic", "fast_slam2/GlobalPose")
WAYPOINTS_TOPIC = rospy.get_param("/planner/waypoints_topic", "waypoints_topic")
REAL_TIME_GLOBAL_PLOTTING = rospy.get_param("/planner/real_time_global_plotting", True)
MAX_DISTANCE_OF_VIEW = rospy.get_param("/planner/max_distance_of_view", 15)

rospy.init_node('path_planner')
waypoints_pub = rospy.Publisher(WAYPOINTS_TOPIC, Float64MultiArray, queue_size=1)


"""For Testing"""
YELLOW_CONE_STYLE = rospy.get_param("/planner/yellow_cone_style", 1)
BLUE_CONE_STYLE = rospy.get_param("/planner/blue_cone_style", 0)
ORANGE_CONE_STYLE = rospy.get_param("/planner/orange_cone_style", 2)
BIG_CONE_STYLE = rospy.get_param("/planner/big_cone_style", 3)
UNKNOWN_CONE_STYLE = rospy.get_param("/planner/unknown_cone_style", 4)


while not rospy.is_shutdown():
    
    """ Subscribing on CarPose and ConeMap Starts here """
    
    car_pose: Odometry = rospy.wait_for_message(POSE_TOPIC, Odometry)
    landmark_array: LandmarkArray = rospy.wait_for_message(MAP_TOPIC, LandmarkArray)

    map_object = Map(landmark_array.landmarks, car_pose.pose.pose) #map object, totally unrelated to python's map function

    best_path = map_object.get_path()
    waypoint: Waypoint
    if (best_path == None):
        print("No Path Found")
    else:
        best_path = list(map(lambda R: (R.x, R.y), best_path.waypoints))

    """ Subscribing on CarPose and ConeMap Ends here """
    

    if (best_path != None):
        output_path = Float64MultiArray()
        layout = MultiArrayLayout()
        dimension = MultiArrayDimension()
        dimension.label = "best_path"
        dimension.size = len(best_path)
        dimension.stride = len(best_path)
        layout.data_offset = 0
        layout.dim = [dimension]
        output_path.layout = layout
        output_path.data = best_path
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
            plt.plot(np.array(best_path)[:,0], np.array(best_path)[:,1], 'x')    #Goal Midpoint
        plt.plot(car_pose.pose.pose.position.x, car_pose.pose.pose.position.y, 'D')        #Current Pose
        plt.pause(0.001)
        plt.cla()

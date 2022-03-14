#! /usr/bin/env python3
import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry, Path as navPath
from asurt_msgs.msg import LandmarkArray

from map import Map
from path import Path

MAP_TOPIC = rospy.get_param("planner/map_topic", "/planner/cones")
POSE_TOPIC = rospy.get_param("planner/pose_topic", "/planner/pose")
WAYPOINTS_TOPIC = rospy.get_param("planner/waypoints_topic", "/planner/path")

BLUE_CONE_STYLE = rospy.get_param("planner/blue_cone_style", 0)
YELLOW_CONE_STYLE = rospy.get_param("planner/yellow_cone_style", 1)
ORANGE_CONE_STYLE = rospy.get_param("planner/orange_cone_style", 2)
BIG_CONE_STYLE = rospy.get_param("planner/big_cone_style", 3)
UNKNOWN_CONE_STYLE = rospy.get_param("planner/unknown_cone_style", 4)

rospy.init_node('path_planner')

waypoints_pub = rospy.Publisher(WAYPOINTS_TOPIC, navPath, queue_size=1)

while not rospy.is_shutdown():
    
    landmark_array: LandmarkArray = rospy.wait_for_message(MAP_TOPIC, LandmarkArray)
    car_pose: Odometry = rospy.wait_for_message(POSE_TOPIC, Odometry)

    map_object = Map(landmark_array.landmarks, car_pose.pose.pose) #map object, totally unrelated to python's map function
    
    best_path: Path = map_object.get_path()
    output_path = navPath()
    if (best_path == None):
        print("No Path Found")
        output_path.header.frame_id = 'map'
        waypoints_pub.publish(output_path)
    else:
        output_path.header.frame_id = 'map'
        output_path.poses = [PoseStamped(Header(frame_id = 'map'), Pose(Point(x = waypoint.x, y = waypoint.y), Quaternion())) for waypoint in best_path.waypoints]
        waypoints_pub.publish(output_path)
from math import sqrt, cos, sin, radians

import rospy
from asurt_msgs.msg import Landmark
from car_pose import CarPose
YELLOW_CONE_STYLE = rospy.get_param("planner/yellow_cone_style", 1)
BLUE_CONE_STYLE = rospy.get_param("planner/blue_cone_style", 0)
ORANGE_CONE_STYLE = rospy.get_param("planner/orange_cone_style", 2)
BIG_CONE_STYLE = rospy.get_param("planner/big_cone_style", 3)
UNKNOWN_CONE_STYLE = rospy.get_param("planner/unknown_cone_style", 4)

class Cone:
    x: float = 0
    y: float = 0
    color: int = 4
    yellow_probability: float = 0.4
    blue_probability: float = 0.4
    orange_probability: float = 0.2

    #Public Functions
    def __init__(self, landmark: Landmark = None):
        if (landmark != None):
            self.x = landmark.position.x
            self.y = landmark.position.y
            if (landmark.type == BLUE_CONE_STYLE):
                self.blue_probability = 1.0
                self.yellow_probability = 0.0
                self.orange_probability = 0.0
            elif (landmark.type == YELLOW_CONE_STYLE):
                self.blue_probability = 0.0
                self.yellow_probability = 1.0
                self.orange_probability = 0.0
            elif (landmark.type == ORANGE_CONE_STYLE or landmark.type == BIG_CONE_STYLE):
                self.blue_probability = 0.0
                self.yellow_probability = 0.0
                self.orange_probability = 1.0
            self.color = landmark.type

    def get_distance(self, x: float, y: float) -> float:
        return sqrt((self.x - x)**2 + (self.y - y)**2)

    def to_frame(self, pose: CarPose):
        c, s = cos(-1*radians(pose.heading)), sin(-1*radians(pose.heading))
        #Rotation Matrix Applied
        self.x = (c * self.x) - (s * self.y)
        self.y = (s * self.x) + (c * self.y)
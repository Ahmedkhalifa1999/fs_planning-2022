from math import sqrt, cos, sin, radians

from asurt_msgs.msg import Landmark
from car_pose import CarPose

class Cone:
    x: float = 0
    y: float = 0
    color: int = 4
    color_confidence: float = 1

    #Public Functions
    def __init__(self, landmark: Landmark = None):
        if (landmark != None):
            self.x = landmark.position.x
            self.y = landmark.position.y
            self.color = landmark.type
            self.color_confidence = 1

    def get_distance(self, x: float, y: float) -> float:
        return sqrt((self.x - x)**2 + (self.y - y)**2)

    def to_frame(self, pose: CarPose):
        c, s = cos(-1*radians(pose.heading)), sin(-1*radians(pose.heading))
        #Rotation Matrix Applied
        self.x = (c * self.x) - (s * self.y)
        self.y = (s * self.x) + (c * self.y)
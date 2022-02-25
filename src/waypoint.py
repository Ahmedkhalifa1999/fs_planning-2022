from math import cos, sin, atan2, radians, degrees, sqrt

from cone import Cone
from car_pose import CarPose

class Waypoint:
    x: float = 0
    y:float = 0
    heading: float = 0
    right_cone: Cone = Cone()
    left_cone: Cone = Cone()

    #Public Fucntions
    def __init__(self, right_cone: Cone, left_cone: Cone):
        self.x = (right_cone.x + left_cone.x)/2
        self.y = (left_cone.y + right_cone.y)/2
        self.right_cone = right_cone
        self.left_cone = left_cone
        self.heading = self.calculate_heading()

    def to_frame(self, pose: CarPose):
        c, s = cos(-1*radians(pose.heading)), sin(-1*radians(pose.heading))
        #Rotation Matrix Applied to Waypoint coordinates
        self.x = (c * self.x) - (s * self.y)
        self.y = (s * self.x) + (c * self.y)
        #Transforming Cones to local frame
        self.right_cone.to_frame(pose)
        self.left_cone.to_frame(pose)
        self.heading = self.calculate_heading()

    def get_distance(self, x: float, y: float) -> float:
        return sqrt((self.x - x)**2 + (self.y - y)**2)
        
    def getWidth(self) -> float:
        return self.right_cone.get_distance(self.left_cone.x, self.left_cone.y)

    #Private Functions
    def calculate_heading(self) -> float:
        perpendicular_heading_x: float = self.right_cone.x - self.left_cone.x
        perpendicular_heading_y: float = self.right_cone.y - self.left_cone.y
        return 90 + degrees(atan2(perpendicular_heading_y,perpendicular_heading_x))
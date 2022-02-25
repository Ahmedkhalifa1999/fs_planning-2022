from math import log as ln
from statistics import stdev
from rospy import get_param as get_ros_param

from waypoint import Waypoint
from car_pose import CarPose

PRIOR_WEIGHT = get_ros_param("planner/prior_weight", 1)
LARGEST_ANGLE_CHANGE_WEIGHT = get_ros_param("planner/largest_angle_change_weight", 1)
LARGEST_ANGLE_CHANGE_SETPOINT = get_ros_param("planner/largest_angle_change_setpoint", 0)
LEFT_STANDARD_DEVIATION_WEIGHT = get_ros_param("planner/left_standard_deviation_weight", 1)
LEFT_STANDARD_DEVIATION_SETPOINT = get_ros_param("planner/left_standard_deviation_setpoint", 0)
RIGHT_STANDARD_DEVIATION_WEIGHT = get_ros_param("planner/right_standard_deviation_weight", 1)
RIGHT_STANDARD_DEVIATION_SETPOINT = get_ros_param("planner/right_standard_deviation_setpoint", 0)
TRACK_WIDTH_STANDARD_DEVIATION_WEIGHT = get_ros_param("planner/track_width_standard_deviation_weight", 1)
TRACK_WIDTH_STANDARD_DEVIATION_SETPOINT = get_ros_param("planner/track_width_standard_deviation_setpoint", 0)
NUMBER_OF_EDGES_CROSSED_WEIGHT = get_ros_param("planner/number_of_edges_crossed_weight", 100)
NUMBER_OF_EDGES_CROSSED_SETPOINT = get_ros_param("planner/number_of_edges_crossed_setpoint", 5)
PATH_LENGTH_WEIGHT = get_ros_param("planner/path_length_weight", 20)
PATH_LENGTH_SETPOINT = get_ros_param("planner/path_length_setpoint", 20)

class Path:
    waypoints: list = list()

    #Public Functions
    def __init__(self, waypoints = list()):
        self.waypoints = waypoints

    def add_waypoint(self, waypoint: Waypoint):
        self.waypoints.append(waypoint)

    def get_posterior(self) -> float:
        return (-1 * PRIOR_WEIGHT * self.get_cost()) + (self.get_log_likelihood())

    def get_last_waypoint(self) -> Waypoint:
        return self.waypoints[-1]
    
    #Private Functions
    def get_largest_angle_change(self) -> float: #Assumes waypoint has data member angle
        if (len(self.waypoints) < 2):
            return 0
        largest_angle_change: float = 0
        for i in range(len(self.waypoints) - 1):
            current_angle_change: float = abs(self.waypoints[i].heading - self.waypoints[i+1].heading)
            if (current_angle_change > largest_angle_change):
                largest_angle_change = current_angle_change
        return largest_angle_change

    def get_track_width_standard_deviation(self) -> float:
        if (len(self.waypoints) < 2):
            return 0
        track_widths: list = list()
        for waypoint in self.waypoints:
            track_widths.append(waypoint.getWidth())
        return stdev(track_widths)

    def get_left_standard_deviation(self) -> float:
        if (len(self.waypoints) < 3):
            return 0
        distances: list = list()
        for i in range(len(self.waypoints) - 1):
            distances.append(self.waypoints[i].left_cone.get_distance(self.waypoints[i+1].left_cone.x, self.waypoints[i+1].left_cone.y))
        return stdev(distances)

    def get_right_standrad_deviation(self) -> float:
        if (len(self.waypoints) < 3):
            return 0
        distances: list = list()
        for i in range(len(self.waypoints) - 1):
            distances.append(self.waypoints[i].right_cone.get_distance(self.waypoints[i+1].right_cone.x, self.waypoints[i+1].right_cone.y))
        return stdev(distances)
    
    def get_number_of_edges_crossed(self) -> int:
        return len(self.waypoints)

    def get_path_length(self) -> float:
        sum: float = 0
        for i in range(len(self.waypoints) - 1):
            sum = sum + self.waypoints[i].get_distance(self.waypoints[i+1].x, self.waypoints[i+1].y)
        return sum

    def get_cost(self) -> float:
        cost: float = 0
        cost += LARGEST_ANGLE_CHANGE_WEIGHT * ((self.get_largest_angle_change() - LARGEST_ANGLE_CHANGE_SETPOINT) ** 2)
        cost += LEFT_STANDARD_DEVIATION_WEIGHT * ((self.get_left_standard_deviation() - LEFT_STANDARD_DEVIATION_SETPOINT) ** 2)
        cost += RIGHT_STANDARD_DEVIATION_WEIGHT * ((self.get_left_standard_deviation() - RIGHT_STANDARD_DEVIATION_SETPOINT) ** 2)
        cost += TRACK_WIDTH_STANDARD_DEVIATION_WEIGHT * ((self.get_track_width_standard_deviation() - TRACK_WIDTH_STANDARD_DEVIATION_SETPOINT) ** 2)
        cost += NUMBER_OF_EDGES_CROSSED_WEIGHT * ((self.get_number_of_edges_crossed() - NUMBER_OF_EDGES_CROSSED_SETPOINT) ** 2)
        cost += PATH_LENGTH_WEIGHT * ((self.get_path_length() - PATH_LENGTH_SETPOINT) ** 2)
        return cost

    def get_log_likelihood(self) -> float:
        log_likelihood: float = 0
        for waypoint in self.waypoints:
            log_likelihood = ln(waypoint.right_cone.color_confidence) + ln(waypoint.left_cone.color_confidence)
        return log_likelihood
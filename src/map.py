from cmath import inf
from statistics import mean
from numpy import array
from math import sqrt, atan2, degrees
from copy import copy
import rospy
from tf.transformations import euler_from_quaternion
from scipy.spatial import Delaunay
from queue import SimpleQueue

from asurt_msgs.msg import LandmarkArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

from cone import Cone
from waypoint import Waypoint
from path import Path
from car_pose import CarPose

YELLOW_CONE_STYLE = rospy.get_param("planner/yellow_cone_style", 1)
BLUE_CONE_STYLE = rospy.get_param("planner/blue_cone_style", 0)
ORANGE_CONE_STYLE = rospy.get_param("planner/orange_cone_style", 2)
BIG_CONE_STYLE = rospy.get_param("planner/big_cone_style", 3)
UNKNOWN_CONE_STYLE = rospy.get_param("planner/unknown_cone_style", 4)

CONE_FIELD_OF_VIEW = rospy.get_param("planner/cone_field_of_view", 180)
CONE_DISTANCE = rospy.get_param("planner/cone_distance", 20)
WAYPOINT_FIELD_OF_VIEW = rospy.get_param("planner/waypoint_field_of_view", 180)
WAYPOINT_DISTANCE = rospy.get_param("planner/waypoint_distance", 6)
MAX_SEARCH_ITERATIONS = rospy.get_param("planner/max_search_iterations", 10)
PATH_QUEUE_LIMIT = rospy.get_param("planner/path_queue_limit", 8)
MAX_WAYPOINTS_PER_PATH = rospy.get_param("planner/max_waypoints_per_path", 8)

class Map:
    waypoints: list = list() #Private
    pose: CarPose = CarPose() #Private

    #Public Functions
    def __init__(self, landmarks: LandmarkArray, pose: Pose):
        cones: list = [Cone(landmark) for landmark in landmarks]
        self.pose.x = pose.position.x
        self.pose.y = pose.position.y

        #Temporary for IPG
        #self.pose.heading = pose.position.z
        self.pose.heading = self.quatrenion_to_heading(pose.orientation)
        
        
        cones = self.filter_local(cones, self.pose, CONE_FIELD_OF_VIEW, CONE_DISTANCE)
        self.waypoints = self.triangulate(cones, self.pose)


    def get_path(self) -> Path:
        if (len(self.waypoints) == 0):
            return None
        paths: SimpleQueue = SimpleQueue() #queue to store possible paths
        #Get waypoints in range of car (at appropriate distance and appropriate angle) to use as starting point for path search
        starting_waypoints: list = self.filter_local(self.waypoints, self.pose, WAYPOINT_FIELD_OF_VIEW, WAYPOINT_DISTANCE)
        if(len(starting_waypoints) == 0):
            starting_waypoints: list = self.filter_local(self.waypoints, self.pose, WAYPOINT_FIELD_OF_VIEW, WAYPOINT_DISTANCE * 2)
        waypoint: Waypoint
        for waypoint in starting_waypoints:
            new_path: Path = Path([waypoint])
            paths.put(new_path) #Enqueue new path starting from waypoint
        for _ in range(MAX_SEARCH_ITERATIONS):
            no_new_paths: bool = True
            size: int = paths.qsize()
            for _ in range(size): #Dequeue current paths and add waypoints to them
                current_path: Path = paths.get()
                if (len(current_path.waypoints) > MAX_WAYPOINTS_PER_PATH):
                    paths.put(current_path)
                    continue
                last_waypoint: Waypoint = current_path.get_last_waypoint() #Get last waypoint of path
		        #Get waypoints in range of last waypoint (as if car reached that waypoint)
                current_possible_waypoints: list = self.filter_local(self.waypoints, last_waypoint, WAYPOINT_FIELD_OF_VIEW, WAYPOINT_DISTANCE)
		        #Build new paths each corresponding to a taking one of the waypoints as next and enqueue them in paths queue
                for waypoint in current_possible_waypoints:
                    if (waypoint not in current_path.waypoints):
                        new_path: Path = Path(copy(current_path.waypoints))
                        new_path.add_waypoint(waypoint)
                        paths.put(new_path)
                        no_new_paths = False
                if (no_new_paths):
                    paths.put(current_path)
            if (paths.qsize() > PATH_QUEUE_LIMIT):
                paths = self.prune_paths(paths) #Prune paths queue to improve performance by removing least likely paths
            if (no_new_paths):
                break


        #Finding path with highest posterior
        best_path: Path = None
        highest_posterior: float = -inf #Calculate posterior of path according to cost function representing prior and likelihood
        while(not paths.empty()):
            current_path: Path = paths.get()
            current_posterior: float = current_path.get_posterior()
            if (current_posterior > highest_posterior):
                    best_path = current_path
                    highest_posterior = current_posterior
        

        return best_path

    #Private Functions
    @staticmethod
    #Filters passed landmarks(cones or waypoints) to those in view of pose, returns array containing landmarks in view only
    #Returned landmarks are in their original frame
    #Assumes passed landmarks are in frame of pose
    def filter_local(landmarks: list, pose: CarPose, field_of_view: float, distance: float) -> list:
        filtered_landmarks = list()
        landmark: Waypoint
        for landmark in landmarks:
            landmark_distance = sqrt(((landmark.x - pose.x) ** 2) + ((landmark.y - pose.y) ** 2))
            heading_to_landmark = degrees(atan2(landmark.y - pose.y, landmark.x - pose.x))
            heading_difference = abs(heading_to_landmark - pose.heading)
            if (heading_difference > 180):
                heading_difference = 360 - heading_difference
            if ((0.1 < landmark_distance) and (landmark_distance < distance)):
                if (heading_difference < (field_of_view/2)):
                    filtered_landmarks.append(landmark)
        return filtered_landmarks
                

    
    @staticmethod
    def triangulate(landmarks: list, pose: CarPose) -> list:
        cone_array = [[landmark.x, landmark.y] for landmark in landmarks]
        triangulation = Delaunay(cone_array)
        simplices = triangulation.simplices
        edges = list()
        simplex: array
        for simplex in simplices:
            if (not Map.edge_in_edges(edges, (simplex[0], simplex[1]))):
                edges.append((simplex[0], simplex[1]))
            if (not Map.edge_in_edges(edges, (simplex[0], simplex[2]))):
                edges.append((simplex[0], simplex[2]))
            if (not Map.edge_in_edges(edges, (simplex[1], simplex[2]))):
                edges.append((simplex[1], simplex[2]))
        waypoints = list()
        edge: tuple
        
        for edge in edges:
            landmark1: Cone = landmarks[edge[0]]
            landmark2: Cone = landmarks[edge[1]]
            if ((landmark1.color == BLUE_CONE_STYLE or landmark1.color == YELLOW_CONE_STYLE) and (landmark2.color == BLUE_CONE_STYLE or landmark2.color == YELLOW_CONE_STYLE) and (landmark1.color != landmark2.color)):
                if (landmark1.color == YELLOW_CONE_STYLE):
                    waypoints.append(Waypoint(right_cone = landmark1, left_cone = landmark2))
                else:
                    waypoints.append(Waypoint(right_cone = landmark2, left_cone = landmark1))
        
                    
        if(len(waypoints) > 3):
            return waypoints
        
        #Add extra waypoints constructed from like-colored cones
        blue_cones = list()
        yellow_cones = list()
        landmark: Cone
        for landmark in landmarks:
            if (landmark.color == BLUE_CONE_STYLE):
                blue_cones.append(landmark)
            elif (landmark.color == YELLOW_CONE_STYLE):
                yellow_cones.append(landmark)

        closest_blue_cone: Cone = None
        for cone in blue_cones:
                min_distance = inf
                cone_distance = sqrt(((cone.x - pose.x) ** 2) + ((cone.y - pose.y) ** 2))
                if (cone_distance < min_distance):
                    min_distance = cone_distance
                    closest_blue_cone = cone
        if (closest_blue_cone != None):
            blue_cones.remove(closest_blue_cone)

        closest_yellow_cone: Cone = None
        for cone in yellow_cones:
                min_distance = inf
                cone_distance = sqrt(((cone.x - pose.x) ** 2) + ((cone.y - pose.y) ** 2))
                if (cone_distance < min_distance):
                    min_distance = cone_distance
                    closest_yellow_cone = cone
        if (closest_yellow_cone != None):
            yellow_cones.remove(closest_yellow_cone)

        while (len(blue_cones) > 0):
            second_blue_cone: Cone
            for cone in blue_cones:
                min_distance = inf
                cone_distance = sqrt(((cone.x - closest_blue_cone.x) ** 2) + ((cone.y - closest_blue_cone.y) ** 2))
                if (cone_distance < min_distance):
                    min_distance = cone_distance
                    second_blue_cone = cone
            #Construct virtual cone
            virtual_yellow_cone = Cone()
            vector_x = second_blue_cone.x - closest_blue_cone.x
            vector_y = second_blue_cone.y - closest_blue_cone.y
            vector_mag = sqrt((vector_x ** 2) + (vector_y ** 2))
            virtual_yellow_cone.x = closest_blue_cone.x - (3 * (vector_y / vector_mag))
            virtual_yellow_cone.y = closest_blue_cone.y + (3 * (vector_x / vector_mag))
            virtual_yellow_cone.color = YELLOW_CONE_STYLE
            virtual_yellow_cone.color_confidence = 1.0 
            waypoints.append(Waypoint(right_cone = virtual_yellow_cone, left_cone = closest_blue_cone))
            blue_cones.remove(second_blue_cone)
            closest_blue_cone = second_blue_cone

        while (len(yellow_cones) > 0):
            second_yellow_cone: Cone
            for cone in yellow_cones:
                min_distance = inf
                cone_distance = sqrt(((cone.x - closest_yellow_cone.x) ** 2) + ((cone.y - closest_yellow_cone.y) ** 2))
                if (cone_distance < min_distance):
                    min_distance = cone_distance
                    second_yellow_cone = cone
            #Construct virtual cone
            virtual_blue_cone = Cone()
            vector_x = second_yellow_cone.x - closest_yellow_cone.x
            vector_y = second_yellow_cone.y - closest_yellow_cone.y
            vector_mag = sqrt((vector_x ** 2) + (vector_y ** 2))
            virtual_blue_cone.x = closest_yellow_cone.x + (3 * (vector_y / vector_mag))
            virtual_blue_cone.y = closest_yellow_cone.y - (3 * (vector_x / vector_mag))
            virtual_blue_cone.color = BLUE_CONE_STYLE
            virtual_blue_cone.color_confidence = 1.0 
            waypoints.append(Waypoint(right_cone = closest_yellow_cone, left_cone = virtual_blue_cone))
            yellow_cones.remove(second_yellow_cone)
            closest_yellow_cone = second_yellow_cone
            
        return waypoints

    @staticmethod
    def quatrenion_to_heading(orientation: Quaternion) -> float:
        quatrenion = (orientation.x, orientation.y, orientation.z, orientation.w)
        return degrees(euler_from_quaternion(quatrenion)[2])

    @staticmethod
    def make_local(landmarks: list, pose: CarPose) -> list:
        local_landmarks = copy.deepcopy(landmarks)
        landmark: Waypoint
        for landmark in local_landmarks:
            landmark.to_frame(pose)
        return local_landmarks

    @staticmethod
    def prune_paths(paths: SimpleQueue) -> SimpleQueue:
        paths_list = list()
        pruned_paths = SimpleQueue()
        while(not paths.empty()):
            paths_list.append(paths.get())
        posteriors = [path.get_posterior() for path in paths_list]
        mean_posterior = mean(posteriors)
        for i in range(len(posteriors)):
            if (posteriors[i] > mean_posterior):
                pruned_paths.put(paths_list[i])
        if (pruned_paths.qsize() > PATH_QUEUE_LIMIT):
            pruned_paths = Map.prune_paths(pruned_paths)
        return pruned_paths

    @staticmethod
    def edge_in_edges(edges: list(), edge: tuple()) -> bool:
        list_edge: tuple
        for list_edge in edges:
            if ((list_edge[0] == edge[0] and list_edge[1] == edge[1]) or (list_edge[1] == edge[0] and list_edge[0] == edge[1])):
                return True
        return False

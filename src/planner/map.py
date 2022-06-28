import numpy as np
from time import time
from scipy.spatial import Delaunay
from copy import copy, deepcopy
from .consts import *
from .cone import Cone
from .waypoint import Waypoint
from .path import Path

class Map:
    def __init__(self, landmarks):
        cones = [Cone(landmark) for landmark in landmarks]
        
        cones = self.filter_local(cones, CONE_FIELD_OF_VIEW, CONE_DISTANCE)
        
        self.waypoints = self.triangulate(cones)
        self.cones = np.array([[cone.x,cone.y] for cone in cones])
        self.color_probs = np.array([cone.color_probs for cone in cones])
        self.searched_paths = []
        
    def get_path(self) -> Path:
        if len(self.waypoints) == 0 or self.cones.shape[0]==0:
            return None  # To fix
            
        paths = []
        costs = []
       
        starting_waypoints: list = [Waypoint(0,0)]
        
        for waypoint in starting_waypoints:
            new_path = Path(self.cones, self.color_probs, [waypoint])
            paths.append(new_path)
            costs.append(np.inf)
        cost_time = 0
        c = 0

        for _ in range(MAX_SEARCH_ITERATIONS):
            self.searched_paths = []
            no_new_paths = True
            size = len(paths)
            for _ in range(size):  # Dequeue current paths and add waypoints to them
                current_path = paths[0]
                paths = paths[1:]
                cost = costs[0]
                costs = costs[1:]
                if len(current_path.waypoints) > MAX_WAYPOINTS_PER_PATH:
                    paths.append(current_path) # TODO: can put in another list to prevent reiterating
                    costs.append(cost)
                    continue
                last_waypoint: Waypoint = current_path.get_last_waypoint()
                
                current_possible_waypoints: list = self.filter_local(self.waypoints, WAYPOINT_FIELD_OF_VIEW, WAYPOINT_DISTANCE, last_waypoint)
                
                added_point = False
                for waypoint in current_possible_waypoints:
                    if waypoint not in current_path.waypoints:
                        #new_path: Path = Path(self.cones, self.color_probs, deepcopy(current_path.waypoints))
                        new_path = deepcopy(current_path)
                        #if len(paths)
                        tic = time()
                        new_path.add_waypoint(waypoint.x, waypoint.y)
                        toc = time()
                        
                        new_cost = new_path.get_cost()[0]
                        
                        cost_time += (toc-tic)*1000
                        c += 1

                        
                        if len(paths) > PATH_QUEUE_LIMIT:
                            if new_cost > np.max(costs):
                                continue
                            else:
                                max_idx = np.argmax(costs)
                                self.searched_paths.append(copy(paths[max_idx]))
                                del paths[max_idx]
                                del costs[max_idx]
                        no_new_paths = False
                        added_point = True
                        paths.append(new_path)
                        costs.append(new_cost)
                if not added_point:
                    paths.append(current_path)
                    costs.append(cost)
            if no_new_paths:
                break

        best_path = paths[np.argmin(costs)]
        #print(best_path.get_cost(verbose=True)[3:5])
        # print("-----------")
        # print(c, cost_time, cost_time/c)

        return best_path

    @staticmethod
    def filter_local(landmarks: list, field_of_view: float, distance: float, pose=None) -> list:
        x, y, heading = 0, 0, 0
        if not pose is None:
            x, y, heading = pose.x, pose.y, pose.heading
        filtered_landmarks = []
        
        for landmark in landmarks:
            landmark_distance = np.sqrt(((landmark.x-x) ** 2) + ((landmark.y-y) ** 2))
            heading_to_landmark = np.arctan2(landmark.y-y, landmark.x-x)
            heading_diff = abs(heading_to_landmark-heading - np.pi)%(2*np.pi) - np.pi  # abs + -pi to pi
            if landmark_distance < distance and heading_diff*180/3.14 < (field_of_view/2):
                    filtered_landmarks.append(landmark)
        return filtered_landmarks
    
    @staticmethod
    def triangulate(landmarks: list) -> list:
        cone_array = [[landmark.x, landmark.y] for landmark in landmarks]
        waypoints_obj, waypoints_arr = [], []
        try:
            triangulation = Delaunay(cone_array)
            simplices = triangulation.simplices
            edges = []
            for simplex in simplices:
                if (not Map.edge_in_edges(edges, (simplex[0], simplex[1]))):
                    edges.append((simplex[0], simplex[1]))
                if (not Map.edge_in_edges(edges, (simplex[0], simplex[2]))):
                    edges.append((simplex[0], simplex[2]))
                if (not Map.edge_in_edges(edges, (simplex[1], simplex[2]))):
                    edges.append((simplex[1], simplex[2]))

            for edge in edges:
                landmark1: Cone = landmarks[edge[0]]
                landmark2: Cone = landmarks[edge[1]]
                x = (landmark1.x + landmark2.x)/2
                y = (landmark1.y + landmark2.y)/2
                w = np.array([x,y])
                dist = np.sqrt((x-landmark1.x)**2+(x-landmark1.y)**2)
                dists2 = np.linalg.norm(np.array(cone_array).reshape(-1,2)-w.reshape(1,2), axis=1)
                # Add dists3
                m = np.min(dists2)
                if dist < TRACKWIDTH/2 + 2 and m > DIST_TRIANGULATE2:
                    waypoints_obj.append(Waypoint(x, y))
                    waypoints_arr.append([x,y])
        except:
            pass
        directions = np.array([[0,1],[0,-1],[1,0],[-1,0],[np.sqrt(2)/2,np.sqrt(2)/2],[-np.sqrt(2)/2,np.sqrt(2)/2],[np.sqrt(2)/2,-np.sqrt(2)/2],[-np.sqrt(2)/2,-np.sqrt(2)/2]]).reshape(1,8,2)
        
        new_waypoints = np.array(cone_array).reshape(-1,1,2) + directions*NEW_POINTS_RADIUS
        new_waypoints = new_waypoints.reshape(-1,1,2)
        
        for w in new_waypoints:
            if len(waypoints_arr)==0:
                waypoints_obj.append(Waypoint(w[0][0], w[0][1]))
                waypoints_arr.append([w[0][0],w[0][1]])
            else:
                dists = np.linalg.norm(np.array(waypoints_arr).reshape(-1,2)-w.reshape(1,2), axis=1)
                dists2 = np.linalg.norm(np.array(cone_array).reshape(-1,2)-w.reshape(1,2), axis=1)
                m1, m2 = np.min(dists), np.min(dists2)
                m = np.min([m1,m2])
                if m1 > DIST_TRIANGULATE and m2 > DIST_TRIANGULATE2:
                    waypoints_obj.append(Waypoint(w[0][0], w[0][1]))
                    waypoints_arr.append([w[0][0],w[0][1]])
            
        return waypoints_obj
    def edge_in_edges(edges: list(), edge: tuple()) -> bool:
        list_edge: tuple
        for list_edge in edges:
            if ((list_edge[0] == edge[0] and list_edge[1] == edge[1]) or (list_edge[1] == edge[0] and list_edge[0] == edge[1])):
                return True
        return False

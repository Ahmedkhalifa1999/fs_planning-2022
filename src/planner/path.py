import numpy as np

from .consts import *
from .waypoint import Waypoint
import time
from copy import deepcopy
class Path:
    def __init__(self, cones, color_probs, waypoints = None):
        self.cones = cones.astype(float)
        self.color_probs = color_probs
        self.waypoints = waypoints
        if waypoints is None:
            self.waypoints = []
        self.left_dists = []
        self.right_dists = []
        self.left_units = []
        self.undet_left = 0
        self.undet_right = 0

    def add_waypoint(self, x: float, y:float):
        waypoint = Waypoint(x, y)
        
        if len(self.waypoints) == 0:  # Shouldn't arrive here
            waypoint.update_heading_prev(Waypoint(0,0))
            self.waypoints.append(waypoint)
        else:
            waypoint.update_heading_prev(self.waypoints[-1])
            self.waypoints[-1].update_heading_next(waypoint)
            self.waypoints.append(waypoint)
            cones = np.array(self.cones)
            w0 = self.waypoints[-2]
            w1 = self.waypoints[-1]
            s,b, left_unit, bound = self.get_slope_and_intercept(w0, w1)
            dot_products = (cones-np.array([w0.x, w0.y]).reshape(1,2))@left_unit.reshape(2,1)
            self.left_units.append(left_unit)
            left_idx = np.where(dot_products>0)[0]
            left_cones = cones[left_idx]
            if left_cones.shape[0]==0:
                self.undet_left += 1
            else:
                new_left_widths = self.get_dist(left_cones, s, b, bound, [w0,w1])
                self.left_dists.append(new_left_widths)
            
            right_idx = np.where(dot_products<=0)[0]
            right_cones = cones[right_idx]
            if right_cones.shape[0]==0:
                self.undet_right += 1
            else:
                new_right_widths = self.get_dist(right_cones, s, b, bound, [w0,w1])
                self.right_dists.append(new_right_widths)

    def get_last_waypoint(self) -> Waypoint:
        return self.waypoints[-1]
    
    def get_largest_angle_change(self) -> float:
        if len(self.waypoints) == 0:
            return 0
        total_change = self.waypoints[0].imm_heading
        largest_angle_change: float = np.abs(self.waypoints[0].imm_heading)
        for i in range(len(self.waypoints) - 1):
            current_angle_change: float = np.abs(self.waypoints[i].imm_heading - self.waypoints[i+1].imm_heading)
            total_change += self.waypoints[i].imm_heading - self.waypoints[i+1].imm_heading
            if current_angle_change > largest_angle_change:
                largest_angle_change = current_angle_change
        #print(total_change)
        return largest_angle_change
    
    def get_path_length(self) -> float:
        if len(self.waypoints) == 0:
            return 0
        total_length: float = self.waypoints[0].get_distance(0,0)  # Initial distance from car
        max_dist = self.waypoints[0].get_distance(0,0)
        for i in range(len(self.waypoints) - 1):
            dist = self.waypoints[i].get_distance(self.waypoints[i+1].x, self.waypoints[i+1].y)
            total_length = total_length + dist
            if dist > max_dist:
                max_dist = dist
        return total_length, max_dist
    
    def get_slope_and_intercept(self, w1, w2):
        x1, x2 = w1.x, w2.x
        y1, y2 = w1.y, w2.y
        heading = np.arctan2(y2-y1,x2-x1)
        left_unit_n = np.array([-1*np.sin(heading), np.cos(heading)]).reshape(2,1)
        
        denom = (x2-x1)
        if denom <1e-4: # To fix
            denom += 1e-4
        slope = (y2-y1)/denom

        b = y2 - slope*x2
        
        return slope, b, left_unit_n, [np.min([x1,x2]), np.max([x1,x2]), np.min([y1,y2]), np.max([y1,y2])]

    def get_dist(self, cones, s, b, bound, waypoints):
        if cones.shape[0]==0:
            return TRACKWIDTH/2-ERROR_NO_DETECT
        px = cones[:,0]
        py = cones[:,1]
        proj_x = (s*(py-b)+px)/(s*s+1)
        proj_y = s*proj_x + b

        proj = np.array([proj_x, proj_y]).transpose()
        cones_idx1 = np.logical_and(bound[0]<proj_x, proj_x<bound[1])
        cones_idx2 = np.logical_and(bound[2]<proj_y, proj_y<bound[3])
        cones_idx = np.logical_or(cones_idx1, cones_idx2)
        cones_idx = np.where(cones_idx)[0]
        cones2 = cones[cones_idx]
        w1, w2 = waypoints
        if cones2.shape[0]==0:
            x1, x2, y1, y2 = w1.x, w2.x, w1.y, w2.y
            w1 = np.array([x1,y1]).reshape(1,2)
            w2 = np.array([x2,y2]).reshape(1,2)
            dists1 = np.linalg.norm(w1-cones, axis=1)
            dists2 = np.linalg.norm(w2-cones, axis=1)
            m1, m2 = np.min(dists1), np.min(dists2)
            return np.min([m1,m2])
        else:
            dists = np.linalg.norm(proj[cones_idx]-cones2, axis=1)
            
            return np.min(dists)

    def color_cost(self, indices, is_left=True):
        indices = indices[indices>-1]
        
        if indices.shape[0]==0:
            return 0
        colors = np.log(1-self.color_probs[indices])
        c_idx = 0
        nc_idx = [1,2]
        if not is_left:
            c_idx = 1
            nc_idx = [0,2]
        return np.sum(colors[:,c_idx]-0.5*colors[:,nc_idx[0]]-0.5*colors[:,nc_idx[1]])
    
    def get_indices(self, normals):
        cones = self.cones
        waypoints = []
        for w in self.waypoints:
            waypoints.append([w.x, w.y])
        waypoints = np.array(waypoints)
        
        right_side = np.ones(cones.shape[0])
        last_angle = None
        for w, n in zip(waypoints,normals):
            dot_products = (cones-w.reshape(1,2))@n.reshape(2,1)
            n2 = n.reshape(-1)
            heading = np.arctan2(n2[1], n2[0]) - np.pi/2
            if heading < -np.pi:
                heading += 2*np.pi
            if heading > np.pi:
                heading -= 2*np.pi
            if last_angle is None:
                diff = -1
            else:
                diff = heading - last_angle
                
            last_angle = heading
            right_idx = np.where(dot_products<=0)[0]
            if diff > 0: # outside (or)
                right_side[right_idx] = 1
            else: # inside (and)
                inside = np.zeros(cones.shape[0])
                inside[right_idx] = 1
                right_side = right_side*inside
        
        return np.where(right_side==0)[0], np.where(right_side==1)[0]
        
    def get_cost(self) -> float:
        # TODO: normalize each cost
        cost: float = 0
        angle_change = self.get_largest_angle_change()
        cost += LARGEST_ANGLE_CHANGE_WEIGHT * angle_change**2
        if angle_change > 2.5:
            cost += 100
            return cost, 0, 0

        left_dists = deepcopy(self.left_dists)
        right_dists = deepcopy(self.right_dists)
        left_units = deepcopy(self.left_units)
        

        left_idx, right_idx= self.get_indices(left_units)
        # if len(left_dists) > 0 and np.min(left_dists) < 0.5:
        #     cost += 100
        # if  len(right_dists) > 0 and np.min(right_dists) < 0.5:
        #     cost += 100

        left_dists.extend([ERROR_NO_DETECT]*self.undet_left)
        right_dists.extend([ERROR_NO_DETECT]*self.undet_right)
        left_dists = np.array(left_dists)
        right_dists = np.array(right_dists)
        loss_left = np.mean((left_dists-TRACKWIDTH/2)**2)
        loss_right = np.mean((right_dists-TRACKWIDTH/2)**2)
        avg_width_loss = (loss_left+loss_right)/2
        
        
        cost += TRACK_WIDTH_WEIGHT * (avg_width_loss)
        blue_color_cost = self.color_cost(left_idx)
        yellow_color_cost = self.color_cost(right_idx, False)

        # length, max_dist = self.get_path_length()
        # cost += max_dist*MAX_DIST_WEIGHT
        # cost -= length*LENGTH_WEIGHT
        cost += (blue_color_cost + yellow_color_cost)*COLOR_WEIGHT
        return cost, avg_width_loss, angle_change, blue_color_cost, yellow_color_cost, left_dists, right_dists
    
    def __repr__(self):
        return str(self.waypoints)
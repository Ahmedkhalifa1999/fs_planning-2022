import os

import yaml
import numpy as np
from scipy import interpolate

import matplotlib.pyplot as plt

from .spline_interpolator import ArcLengthSpline

class Waypoint:
    def __init__(self,pos):
        self.positions = [pos]
    def add_reading(self,pos):
        self.positions.append(pos)
    def get(self):
        return np.mean(self.positions,axis=0)
        
class WaypointsCleaner:
    '''
    Class for filtered the waypoints recieved from navigation
    Uses NearestNeighbors starting from the car location to find the sequence of waypoints to follow while ignoring very far (outlier) waypoints
    While doing NN, if a point is too close, it is instead average with the last point (i.e. combined together) weighted by the costs of each
    A 3rd order spline is fitted to the extracted points while smoothness is increased (a parameter)
    A set of equally distanced N (self.num_waypoints_return) waypoints are extracted from the spline and returned
    '''
    def __init__(self, to_rear, param_path=None):
        self.to_rear = np.array(to_rear)

        if param_path is None:
            dirname = os.path.dirname(__file__)
            param_path = os.path.join(dirname, 'params.yaml')
            
        with open(param_path,"r") as f:
            params = yaml.safe_load(f)

        for key in params.keys():
            setattr(self,key,params[key])
        self.reset()
        
    def reset(self):
        self.curr_pos = np.zeros(2)
        self.smooth_prev = np.zeros((1,2))
        self.curr_heading = 0
        self.prev_waypoints = []
        self.prev_done_waypoints = []
        self.passed_waypoints = []

    def update_position(self,curr_pos, curr_heading):
        '''
        Updates current global position and heading of the vehicle
        
        Parameters
        ----------
        curr_pos: list of float, shape=2
            Contains the current global location of the vehicle [x (m), y (m)]
        
        curr_heading: float (rad)
            Contains the current heading of the vehicle in radians
        '''
        new_pos = np.array(curr_pos)
        if np.linalg.norm(new_pos-self.curr_pos) > 2:
            self.reset()
        self.curr_pos = new_pos
        self.curr_heading = curr_heading
        for idx,waypoint in enumerate(self.prev_waypoints):
            position = waypoint.get()
            if np.linalg.norm(position-self.curr_pos)<1:
                self.prev_done_waypoints.append(idx)
        
    def add_waypoints(self,waypoints):
        '''
        Adds a set of waypoints to the list of previously seen waypoints used for fitting a polynomial
        
        Parameters
        ----------
        waypoints: list of list of float, shape=(N,2)
            List of waypoints of the current frame, each row is a waypoint [x (m), y (m)]
        '''
        # Transform waypoints to global (map) frame
        waypoints = np.array(waypoints)
        s, c = np.sin(self.curr_heading), np.cos(self.curr_heading)
        rot_mat = np.array([[c,-s],[s,c]]) # to check
        waypoints = (rot_mat@waypoints.transpose()).transpose()
        waypoints = np.array(waypoints) + self.curr_pos.reshape(1,2)

        positions = []
        for waypoint in self.prev_waypoints:
            positions.append(waypoint.get())
        positions = np.array(positions)
        
        for waypoint in waypoints:
            if positions.shape[0]>0:
                dists = np.linalg.norm(positions-waypoint,axis=1)  # shape=N
                min_dist, min_dist_idx = np.min(dists), np.argmin(dists)
                if min_dist < self.min_dist_unique:
                    self.prev_waypoints[min_dist_idx].add_reading(waypoint)
                else:
                    self.prev_waypoints.append(Waypoint(waypoint))
            else:
                self.prev_waypoints.append(Waypoint(waypoint))
                
        
    def get_ordered_waypoints(self,waypoints):
        s, c = np.sin(self.curr_heading), np.cos(self.curr_heading)
        rot_mat = np.array([[c,-s],[s,c]]) # to check
        vec_rear = (rot_mat@self.to_rear.transpose()).transpose()
        rear_pos = self.curr_pos + vec_rear
        ordered_waypoints = [rear_pos, self.curr_pos]
        curr_waypoint = self.curr_pos
        while waypoints.shape[0]>0:
            dists = np.linalg.norm(waypoints-curr_waypoint,axis=1)  # shape=N
            min_dist, min_dist_idx = np.min(dists), np.argmin(dists)
            if min_dist > self.euclidean_dist_outlier:
                break
                
            curr_waypoint = np.copy(waypoints[min_dist_idx])
            ordered_waypoints.append(waypoints[min_dist_idx].tolist())
            
            waypoints = np.delete(waypoints,min_dist_idx,axis=0)
        return np.array(ordered_waypoints)
            
    
    def fit_spline(self,waypoints):
        k = 2
        if waypoints.shape[0]<4:
            k = waypoints.shape[0]-1
            
        if k<1:
            return waypoints
            
        tck, u = interpolate.splprep(waypoints.T,w = np.ones(waypoints.shape[0]), s=self.interpolate_smoothing, k=k)  # Cubic spline
        unew = np.linspace(0,1,self.num_interpolation_points)
        new_waypoints = np.array(interpolate.splev(unew, tck)).T
        return new_waypoints
        
    def get_waypoints(self):
        '''
        
        Returns
        -------
        waypoints: list of list of float, shape=(self.num_waypoints_return, 2)
            List of waypoints filtered that should be used by navigation
        '''
        positions = []
        num_readings = []
        
        car_forward_vector = np.array([np.cos(self.curr_heading),np.sin(self.curr_heading)])
        
        for idx,waypoint in enumerate(self.prev_waypoints):
            waypoint_pos = waypoint.get()
            if idx not in self.prev_done_waypoints and len(waypoint.positions)>self.min_observations_before_using_waypoint:
                is_front = np.dot(car_forward_vector,waypoint_pos-self.curr_pos)
                
                if is_front>0 or not self.filter_behind_car:
                    positions.append(waypoint_pos)
                    num_readings.append(np.clip(len(waypoint.positions),0,self.max_num_readings))
            
        # Readings filter
        if len(positions)==0:
            return np.array([])
        mean_num_readings = np.mean(num_readings)
        
        waypoints_to_use = []
        for pos, read in zip(positions, num_readings):
            if read>=mean_num_readings-self.mean_num_readings_slack:
                waypoints_to_use.append(pos)
        waypoints_to_use = np.array(waypoints_to_use)
        
        if waypoints_to_use.shape[0]==0:
            return np.array([])
            
        # TODO: Outlier removal using perpendicular distance to the previously fitted spline
        ordered_waypoints = self.get_ordered_waypoints(waypoints_to_use)

        new_waypoints = self.fit_spline(ordered_waypoints)
        if new_waypoints.shape[0]<4: return new_waypoints
        
        ac_spline = ArcLengthSpline(num_samples=10,arclength_dt=0.1)
        ac_spline.fit_spline(new_waypoints[:,0],new_waypoints[:,1])
        all_waypoints = ac_spline.get_points()
        return all_waypoints

        x,y = ac_spline.evaluate(self.theta_lookahead)
        next_waypoint = np.array([[x,y]])
        smoothed = next_waypoint*(1-self.exp_param) + self.exp_param*self.smooth_prev
        self.smooth_prev = smoothed
        
        
        # Plot
        if self.plot:
            plt.clf()
            plt.xlim([-50,50])
            plt.ylim([-100,100])
            plt.plot(new_waypoints[:,0],new_waypoints[:,1],c='c')
            plt.scatter(waypoints_to_use[:,0],waypoints_to_use[:,1],c='g',marker='x')
            plt.scatter(smoothed[:,0],smoothed[:,1],c='k',marker='^')
            plt.pause(0.05)
        
        return [[*self.curr_pos],smoothed[0]]
        
        

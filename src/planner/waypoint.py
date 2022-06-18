import numpy as np

class Waypoint:
    def __init__(self, x: float, y: float, heading:float=0):
        self.x = x
        self.y = y
        self.heading = heading  # In radians, pi to -pi
        self.imm_heading = heading

    def get_distance(self, x: float, y: float) -> float:
        return np.sqrt((self.x - x)**2 + (self.y - y)**2)

    def update_heading_next(self, next_waypoint) -> float:  # TODO: assert it is run only once
        delta_x: float = next_waypoint.x - self.x
        delta_y: float = next_waypoint.y - self.y
        self.heading = (self.heading+np.arctan2(delta_y,delta_x)) / 2
        if self.heading > np.pi:
            self.heading -= np.pi
        if self.heading < -np.pi:
            self.heading += np.pi
        
    def update_heading_prev(self, prev_waypoint) -> float:
        delta_x: float = self.x - prev_waypoint.x
        delta_y: float = self.y - prev_waypoint.y
        self.heading = np.arctan2(delta_y,delta_x)
        self.imm_heading = self.heading
    
    def __repr__(self):
        return "Waypoint, Pos: ({:.1f},{:.1f}), Heading: {:.2f}\n".format(self.x, self.y, self.heading*180/3.14)
    
    def __eq__(self, other):
        return self.x==other.x and self.y==other.y  
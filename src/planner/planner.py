from .map import Map
from .path import Path
from threading import Lock

from tf_helper import *

mutex = Lock()
class Landmark:
    def __init__(self, x, y, t):
        self.x = x
        self.y = y
        self.color_probs = [0.33, 0.33, 0.33]
        if t==1:
            self.color_probs = [0.8,0.1,0.1]
        elif t==0:
            self.color_probs = [0.1,0.8,0.1]
        
def landmark_objects(landmarks):
    landmark_objs = []
    for landmark in landmarks:
        landmark_objs.append(Landmark(landmark.position.x, landmark.position.y, landmark.type))
    return landmark_objs

class Planner:
    def __init__(self):
        self.landmarks = None

    @mutex_lock(mutex)
    def landmark_cb(self, landmarks):
        self.landmarks = landmark_objects(landmarks.landmarks)

    @mutex_lock(mutex)
    def run(self):
        if self.landmarks is None:
            return None

        map_object = Map(self.landmarks)
        best_path = map_object.get_path()

        # Waypoints Cleaner
        return best_path
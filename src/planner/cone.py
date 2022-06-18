from math import sqrt


class Cone:
    x: float = 0  # In meters
    y: float = 0  # In meters
    color_probs: list = [0.33,0.33,0.33]


    def __init__(self, landmark=None):
        if (landmark != None):
            self.x = landmark.x
            self.y = landmark.y
            self.color_probs = landmark.color_probs

    def get_distance(self, x: float, y: float) -> float:
        return sqrt((self.x - x)**2 + (self.y - y)**2)
#! /usr/bin/env python
class cone:
    x
    y
    confidence
    def __init__(self, x, y, type, confidence): #Takes landmark array, and type of cone desired
        #Initialize data members from landmark array
        pass
    def distance(self):
        #Calculate distance of cone from car (return np array)
        pass
    def getType(self):
        #Calculate 
        pass
    def 


#Create four "cones" objects, left, right, unknown and all (Memory usage?)



# This function assumes that a cone class exists with properties: x, y, z and confidence
import random
import numpy as np

"""
This function takes an array of cone objects as an argument. 
It reduces the confidence of the cones based on the distance to the car with a random variable.
It also adds noise to the poisition of the cone
"""
def add_noise(cones):
    #Reducing confidence based on distance
    cones.confidence = cones.confidence - ((np.random.rand(cones.length) * (cones.distance)) / (50*cones.distance.min()))
    #cones.confidence = (0.1*np.max(cones.distance) * np.random.rand(cones.length)) / ((cones.distance)**2)
    
    for conf in cones.confidence:
        if (conf < 0):
            conf = 0
        if (conf > 1):
            conf = 1
    
    #Adding noise to the poistion of the cones
    cones.x = cones.x + 2*np.random.rand(cones.length)
    cones.y = cones.y + 2*np.random.rand(cones.length)
    
    #Add random cones to array
    max_x = np.max(cones.x)
    max_y = np.max(cones.y)
    for _ in range(1,cones.length):
        if (random.randint(1, 10) == 1):
            #rand_x = random.random() * (cones.x.max()*2);
            #rand_y = random.random() * (cones.y.max()*2);
            #rand_z = cones.z.mean();
            cones.x = np.append(cones.x, random.random() * max_x)
            cones.y = np.append(cones.y, random.random() * max_y)
            cones.confidence = np.append(cones.confidence, 0.75*random.random())
            cones.length = cones.length + 1
            #cones.append(rand_x, rand_y, rand_z, rand.random());

"""

"""
def pre_process(cones):

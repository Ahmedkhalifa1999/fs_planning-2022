# Path-Planning
## Introduction

## Functional Description
1. Node gets a message of LandMarkArray type and uses it to construct a list of landmarks(cones).
2. Landmarks(cones) are used with delaunay triangulation to construct a list of waypoints(midpoints of edges).
3. Waypoints are used to grow a tree of possible paths in a breadth first manner. :heavy_exclamation_mark:**Need to discuss algorithm for doing so to minimize the number of branches searched (Dikjstra? A-star? heuristic?)**:heavy_exclamation_mark:
4. The most likely path of the tree of paths is sent to the control node as a list of waypoints.

## Classes
### Cone
#### Data Members:
* x                   : x position of cone in local frame, frame origin passed to constructor as argument
* y                   : y position of cone in local frame, frame origin passed to constructor as argument
* color               : cone color, can be blue, yellow or unknown
* colorConfidence     : value from 0 to 1 representing confidence that cone is of specified color (Will it be relevant for unknown cones?)
* existenceConfidence : value represneting confidence that cone actually exits (Will it be useful? Not used in AMZ paper)

#### Static Methods:

#### Constructors:
* (landmark, origin) : takes landmark message and origin of local frame as arguments, returns a cone in local frame :red_circle:**Not Implemented Yet**:red_circle:

#### Instance Methods:
* toLocal(origin) : takes origin pose as a tuple in global frame and transforms cone position into local frame of passed origin :red_circle:**Not Implemented Yet**:red_circle:
* getDistance(origin) : takes position as a tuple and returns the distance of the cone from passed position

### Waypoint
#### Data Members:

#### Static Methods:

#### Constructors:

#### Instance Methods:

### Path
#### Data Members:
* waypoints : *ordered* list of waypoints that make up the path

#### Static Methods: 

#### Constructors:
* (waypoints = []) : takes a list of waypoints as an *optional* argument

#### Instance Methods:
* addWaypoint(waypoint) : takes a waypoint object as an argument and adds it to waypoints of path
* getLargestAngleChange() : finds largest angle change within the path and returns it in degress
* getTrackWidthStandardDeviation() : calculates the standrad deviation of track width within the path
* getLeftStandardDeviation() : calculates the standard deviation of distances between left cones
* getRightStandradDeviation() : calculates the standard deviation of distances between right cones
* getNumberOfEdgesCrossed() : returns the number of waypoints in the path
* getPathLength() : calcualtes the length of the path :red_circle:**Not Implemented Yet**:red_circle:
* getCost() : calcualtes the cost parameter used to calcualtes the prior of the path :red_circle:**Not Implemented Yet**:red_circle:
* getPrior() : calculates the prior of the path using cost :red_circle:**Not Implemented Yet**:red_circle:
* getLikelihood() : calculates the likelihood of the path using cone confidences :red_circle:**Not Implemented Yet**:red_circle:
* getPosterior() : calculates the posterior of the path using likelihood and priod :red_circle:**Not Implemented Yet**:red_circle:

## Map:question:
#### Data Members:

#### Static Methods:

#### Constructors:

#### Instance Methods:

## Functions (may not need any)

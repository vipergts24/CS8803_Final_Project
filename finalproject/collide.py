#X_OFFSET = 500
#Y_OFFSET = 300
MIN_X = 248     #240 before correction 
MIN_Y = 111     #105 before correction
MAX_X = 1690    #1696 before correction
MAX_Y = 967     #974 before correction
#OBSTACLE_BOTTOM = (1000/2 - X_OFFSET, Y_OFFSET - 640/2)
OBSTACLE_CENTER = (1000, 542)

from util import distance_between
from math import *

def collision_update(x, y, heading):
    #print x, y
    didCollide = False
    # Hitting Center
    dist = distance_between((x,y), OBSTACLE_CENTER)
    if dist <= 100:
        didCollide = True
        s = sin(heading)
        c = cos(heading)
        # angle of the tangent
        ca = atan2(y-OBSTACLE_CENTER[1], x-OBSTACLE_CENTER[0])
        # reversed collision angle of the robot
        rca = atan2(-s, -c)
        # reflection angle of the robot
        heading = rca + (ca - rca) * 2
    # Hitting Walls
    if x < MIN_X:
        didCollide = True
        heading = pi - heading
        x = MIN_X + 1
    elif x > MAX_X:
        didCollide = True
        heading = pi - heading
        x = MAX_X - 1
    if y < MIN_Y:
        didCollide = True
        heading = -heading #%360
        y = MIN_Y - 1
    elif y > MAX_Y: 
        didCollide = True
        heading = -heading #%360
        y = MAX_Y - 1
    return x,y,heading,didCollide

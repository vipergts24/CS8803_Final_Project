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
    if x < MIN_X or x > MAX_X:#x < MIN_X/2 - X_OFFSET or x > MAX_X/2 - X_OFFSET:
        didCollide = True
        heading = pi - heading #(180 - heading) % 360
    if y < MIN_Y or y > MAX_Y: #y > Y_OFFSET - MIN_Y/2 or y < Y_OFFSET - MAX_Y/2:
        didCollide = True
        heading = -heading #%360
    return heading,didCollide

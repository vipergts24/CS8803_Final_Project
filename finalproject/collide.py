X_OFFSET = 500
Y_OFFSET = 300
MIN_X = 240
MIN_Y = 105
MAX_X = 1696
MAX_Y = 974
OBSTACLE_BOTTOM = (1000/2 - X_OFFSET, Y_OFFSET - 640/2)
OBSTACLE_CENTER = (1000/2 - X_OFFSET, Y_OFFSET - 540/2)

from util import distance_between
from math import degrees

def collision_update(x, y, heading):
    # Hitting Center
    dist = distance_between((x,y), OBSTACLE_CENTER)
    if dist <= 50:
        s = sin(radians(heading))
        c = cos(radians(heading))
        # angle of the tangent
        ca = atan2(y-OBSTACLE_CENTER[1], x-OBSTACLE_CENTER[0])
        # reversed collision angle of the robot
        rca = atan2(-s, -c)
        # reflection angle of the robot
        heading = degrees(rca + (ca - rca) * 2)%360
    # Hitting Walls
    if x < MIN_X/2 - X_OFFSET or x > MAX_X/2 - X_OFFSET:
        heading = (180 - heading) % 360
    if y > Y_OFFSET - MIN_Y/2 or y < Y_OFFSET - MAX_Y/2:
        heading = -heading%360
    return heading

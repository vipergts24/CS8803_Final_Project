from math import *

def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

#Distance function utilized in runaway robot
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def readData(filename):
    """Reads data and returns data as a list of (x,y) tuples"""
    data = []
    with open(filename, 'r') as f:
        for line in f:
            val = line.split(',')
            x = int(val[0])
            y = int(val[1])
            data.append((x,y))
    return data

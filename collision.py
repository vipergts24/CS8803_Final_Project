################################################################################
# CS8803 Summer 2015                                                           #
# Final Project                                                                #
# Authors: Craig Abernethy, Glenn Abernethy, Swapnil Ralhan                    #
################################################################################

#Included matrix module from Runway Robot Project
from matrix import *
import turtle

def collideSetup():
    global minX,minY,maxX,maxY,xOffset, yOffset 
    xOffset, yOffset = 500,300
    minX,minY = 240,105
    maxX,maxY = 1696,974
    window = turtle.Screen()
    window.bgcolor('white')
    box = turtle.Turtle()
    box.color('#AAAAAA')
    box.penup()
    box.goto(minX/2-xOffset,yOffset-minY/2)
    box.pendown()
    box.goto(maxX/2-xOffset,yOffset-minY/2)
    box.goto(maxX/2-xOffset,yOffset-maxY/2)
    box.goto(minX/2-xOffset,yOffset-maxY/2)
    box.goto(minX/2-xOffset,yOffset-minY/2)
    box.penup()
    box.hideturtle()

    obstacle = turtle.Turtle()
    obstacle.color('#AA0000')
    obstacle.penup()
    obstacle.goto(1000/2-xOffset,yOffset-640/2)
    obstacle.pendown()
    obstacle.circle(50)
    obstacle.penup()
    obstacle.hideturtle()
            
    global broken_robot
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.25, 0.25, 0.25)
    broken_robot.up()
    return True

def move_and_collide(x, y, heading, velocity):
    collision = False
    broken_robot.setheading(80)
    broken_robot.goto(x,y)
    broken_robot.pendown()
    while True:
        x,y = broken_robot.position()
        heading = broken_robot.heading()
        if x<minX/2-xOffset or x>maxX/2-xOffset:
            heading = (180 - heading) % 360
            broken_robot.setheading(heading)
        if y>yOffset-minY/2:
            heading = -heading % 360
            broken_robot.setheading(heading)
        else:
            if y< yOffset-maxY/2:
                heading = -heading % 360
                broken_robot.setheading(heading)
        broken_robot.forward(velocity)

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

def main():
    initEKF = None #Initial Extended Kalman Filter Generated from the training_data.txt
    initEKF = 'initialKF.txt'#file we are saving our initial Extended Kalman Filter
    try:
        initEKF = open('initialKF.txt')
        initEKF = matrix(initEKF.read())
    except:
        print 'No initial EKF found: generating new one \n'
        #initEKF = generateInitialEKF(initEKF)    
    testFiles = ['training.txt']
    #testFiles = ['test01.txt','test02.txt','test03.txt','test04.txt','test05.txt','test06.txt','test07.txt','test08.txt','test09.txt','test10.txt']
    for testFile in testFiles:
        data = readData(testFile)
        #TO DO: Process data to generate our EKF
    #TO DO: Predict the next 60 steps
#Run our main routine
#main()
        
collideSetup()
move_and_collide(minX + 100,minY + 100,0,3)

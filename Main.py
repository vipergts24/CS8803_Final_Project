################################################################################
# CS8803 Summer 2015                                                           #
# Final Project                                                                #
# Authors: Craig Abernethy, Glenn Abernethy, Swapnil Ralhan                    #
################################################################################

#Included matrix module from Runway Robot Project
from matrix import *
import turtle
X = None
P = None
measurement_noise = .05

def kalman_filter(measurement,OTHER,X,P):
    # identity matrix
    I = matrix([[1., 0., 0., 0.,0.],
                [0., 1., 0., 0.,0.],
                [0., 0., 1., 0.,0.],
                [0., 0., 0., 1.,0.],
                [0., 0., 0., 0.,1.]])
    #motion update matrix
    H = matrix([[1., 0., 0., 0., 0.],
                [0., 1., 0., 0., 0.]])
    #measurement noise
    R = matrix([[measurement_noise, 0.],
                [0., measurement_noise]])
    #measurement
    Z = matrix([[measurement[0]],
                [measurement[1]]])
    #External Motion
    u = matrix([[0.],
                [0.],
                [0.],
                [0.],
                [0.]])

    #x,y,lastAngle,turningAngle,distance
    lastX = OTHER['lastMeasurement'][0]
    lastY = OTHER['lastMeasurement'][1]
    angle = OTHER['lastAngle']
    turningAngle = OTHER['turningAngle']
    predictedAngle = angle_trunc(angle+turningAngle)
    distance = OTHER['distance']
    #Prediction
    F = matrix([[1.,0.,0.,0.,-sin(predictedAngle)],
               [0.,1.,0.,0.,cos(predictedAngle)],
               [0.,0.,1.,0.,0.],
               [0.,0.,0.,1.,0.],
               [0.,0.,0.,0.,1.]])
    
    X = (F * X) + u    
    P = F * P * F.transpose()
    
    #measurement update
    Y = Z - (H * X)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()

    X = X + (K * Y)
    P = (I - (K * H)) * P
    
    return X,P

#Reads in a textFile and returns a list of tuples containing the coordinates
def readData(textFile):
    data = []
    f = None
    try:
        f = open(textFile,'r')
        window = turtle.Screen()
        window.bgcolor('white')
        broken_robot = turtle.Turtle()
        broken_robot.shape('turtle')
        broken_robot.color('green')
        broken_robot.resizemode('user')
        broken_robot.shapesize(0.25, 0.25, 0.25)
        broken_robot.penup()

        predict_robot = turtle.Turtle()
        predict_robot.shape('turtle')
        predict_robot.color('red')
        predict_robot.resizemode('user')
        predict_robot.shapesize(0.25, 0.25, 0.25)
        predict_robot.penup()
        OTHER = None
        for line in f:
            val = line.split(',')
            x = int(val[0])
            y = int(val[1])
            data.append((x,y))
            print 'Measured: ',x, ',',y,'\n'
            broken_robot.goto(int(x)/2-500, 300-int(y)/2) # flip the y and offset the x
            broken_robot.pendown() # or .stamp()
            global X
            global P
            if X is None:
                #Initial state {x,y,xVelocity,yVelocity}
                X = matrix([[x],
                            [y],
                            [0.],
                            [0.],
                            [0.]])
                #Initial Uncertainty
                P = matrix([[1000.,0.,0.,0.,0.],
                            [0.,1000.,0.,0.,0.],
                            [0.,0.,1000.,0.,0.],
                            [0.,0.,0.,1000.,0.],
                            [0.,0.,0.,0.,1000.]])
                OTHER = {'lastMeasurement':(x,y)}
            angle = atan2(y-OTHER['lastMeasurement'][1],x-OTHER['lastMeasurement'][0])
            angle = angle_trunc(angle)
            if 'lastAngle' not in OTHER:
                OTHER['lastAngle'] = angle
            OTHER['turningAngle'] = angle - OTHER['lastAngle']
            OTHER['distance'] = distance_between((x,y),OTHER['lastMeasurement'])
            X,P = kalman_filter((x,y),OTHER,X,P)
            print 'Predicted: ',X,'\n'
            predict_robot.goto(int(X.value[0][0])/2-500, 300-int(X.value[1][0])/2) # flip the y and offset the x
            predict_robot.pendown() # or .stamp()
    #except:
    #    print 'Failed to read and process file: ',textFile, '\n'
    finally:
        if f is not None:
            f.close()
    return X,P

#Generates an Inital Extended Kalman Filter from the training_data.txt
def generateInitialEKF(writeFile):
    data = readData('training_data.txt')
    #TO DO: Generate EKF and write to file
    EKF = None
    return EKF

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
    testFiles = ['test05.txt']
    #testFiles = ['test01.txt','test02.txt','test03.txt','test04.txt','test05.txt','test06.txt','test07.txt','test08.txt','test09.txt','test10.txt']
    for testFile in testFiles:
        data = readData(testFile)
        #TO DO: Process data to generate our EKF
    #TO DO: Predict the next 60 steps
#Run our main routine
main()

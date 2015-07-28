from matrix import *
from util import *
from collections import deque
import turtle
import time
X = None
P = None

measurement_noise = 0.05
historicalDequeSize = 5 #The maximum number of historical entries we keep using FIFO
historicalMinimum = 4 #The minimum number of historical data required to predict a historical vector
historicalDeque = deque(maxlen=historicalDequeSize)

from collide import collision_update

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
    #print 'last X,Y: ', (lastX,lastY)
    #print 'predictedAngle: ',predictedAngle
    #print 'distance: ', distance
    X.value[0][0] = measurement[0]
    X.value[1][0] = measurement[1]
    X.value[2][0] = angle
    X.value[3][0] = turningAngle
    X.value[4][0] = distance
    #Prediction
    F = matrix([[1.,0.,0.,0.,cos(predictedAngle)],
               [0.,1.,0.,0.,sin(predictedAngle)],
               [0.,0.,1.,1.,0.],
               [0.,0.,0.,1.,0.],
               [0.,0.,0.,0.,1.]])
    #print 'X: ', X
    X = (F * X) + u
    #print 'F: ', F
    #print 'F*X = ', X
    P = F * P * F.transpose()
    
    #measurement update
    Y = Z - (H * X)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()

    newX = X + (K * Y)
    P = (I - (K * H)) * P

    return X,P

def predict(data,visualize):
    if visualize:
        window = turtle.Screen()
        window.bgcolor('white')
        #Boundaries - Gray
        box = turtle.Turtle()
        box.color('#AAAAAA')
        box.penup()
        box.goto(240/2-500,300-105/2)
        box.pendown()
        box.goto(1696/2-500,300-105/2)
        box.goto(1696/2-500,300-974/2)
        box.goto(240/2-500,300-974/2)
        box.goto(240/2-500,300-105/2)
        box.penup()
        box.hideturtle()
        #Center Obstacle - Red
        obstacle = turtle.Turtle()
        obstacle.color('#AA0000')
        obstacle.penup()
        obstacle.goto(1000/2-500,300-642/2)
        obstacle.pendown()
        obstacle.circle(50)
        obstacle.penup()
        obstacle.hideturtle()
        #Measurements - Green
        broken_robot = turtle.Turtle()
        broken_robot.shape('turtle')
        broken_robot.color('green')
        broken_robot.resizemode('user')
        broken_robot.shapesize(0.25, 0.25, 0.25)
        broken_robot.penup()
        #Predictions - Red
        predict_robot = turtle.Turtle()
        predict_robot.shape('turtle')
        predict_robot.color('red')
        predict_robot.resizemode('user')
        predict_robot.shapesize(0.25, 0.25, 0.25)
        predict_robot.penup()
    OTHER = None
    for element in data:
        x,y = element
        if visualize:
            broken_robot.goto(int(x)/2-500, 300-int(y)/2) # flip the y and offset the x
            broken_robot.pendown() # or .stamp()
        global X
        global P
        global historicalDeque
        global historicalDequeSize
        global historicalMinimum
        if X is None:
            #Initial state {x,y,xVelocity,yVelocity}
            X = matrix([[x],
                        [y],
                        [1.],
                        [1.],
                        [1.]])
            #Initial Uncertainty
            P = matrix([[1000.,0.,0.,0.,0.],
                        [0.,1000.,0.,0.,0.],
                        [0.,0.,1000.,0.,0.],
                        [0.,0.,0.,1000.,0.],
                        [0.,0.,0.,0.,1000.]])
            OTHER = {'lastMeasurement':(x,y)}
        else:
            #Calculates the difference between prediction and measurement
            errorPercentX = float((x-X.value[0][0])/x)
            errorPercentY = float((y-X.value[1][0])/y)
            errorPercentage = (abs(errorPercentX) + abs(errorPercentY))*100
        angle = atan2(y-OTHER['lastMeasurement'][1],x-OTHER['lastMeasurement'][0])
        angle = angle_trunc(angle)
        x,y,angle,didCollide = collision_update(x, y, angle)
        OTHER['lastAngle'] = angle
        OTHER['turningAngle'] = angle - OTHER['lastAngle']
        #OTHER['lastAngle'] = angle
        OTHER['distance'] = distance_between((x,y),OTHER['lastMeasurement'])
        X,P = kalman_filter((x,y),OTHER,X,P)
        OTHER['lastMeasurement'] = (x,y)
        if didCollide:#Reset historical
            historicalDeque = deque(maxlen=historicalDequeSize)
        else:
            normalizedVector = (OTHER['distance']*cos(OTHER['lastAngle'] + OTHER['turningAngle']),OTHER['distance']*sin(OTHER['lastAngle'] + OTHER['turningAngle']))
            #normalizedVector = (cos(OTHER['lastAngle'] + OTHER['turningAngle']),sin(OTHER['lastAngle'] + OTHER['turningAngle']))
            if len(historicalDeque) == historicalDequeSize:
                historicalDeque.popleft()
            historicalDeque.append(normalizedVector)
        if visualize:
            predict_robot.goto(int(X.value[0][0])/2-500, 300-int(X.value[1][0])/2) # flip the y and offset the x
            predict_robot.pendown() # or .stamp()
    predictions = []
    #Two Second Predictions - Blue
    #print historicalDeque
    if visualize:
        unknown_robot = turtle.Turtle()
        unknown_robot.shape('turtle')
        unknown_robot.color('blue')
        unknown_robot.resizemode('user')
        unknown_robot.shapesize(0.25, 0.25, 0.25)
        unknown_robot.penup()
    #print historicalDeque
    for i in range(60):
        #print 'deque: ',historicalDeque
        x = int(X.value[0][0])
        y = int(X.value[1][0])
        predictions.append((x,y))
        if len(historicalDeque) > historicalMinimum:#If we have history, use it for our prediction
            xVector = 0
            yVector = 0
            for vector in historicalDeque:
                xVector += vector[0]
                yVector += vector[1]
            xVector = xVector/len(historicalDeque)
            yVector = yVector/len(historicalDeque)
            angle = atan2(yVector,xVector)
            #print 'Historical Vector Angle: ', angle*180/pi
        else:
            angle = atan2(y-OTHER['lastMeasurement'][1],x-OTHER['lastMeasurement'][0])
        angle = angle_trunc(angle)
        x,y,angle,didCollide = collision_update(x, y, angle)
        OTHER['lastAngle'] = angle
        OTHER['turningAngle'] = angle - OTHER['lastAngle']
        #OTHER['distance'] = distance_between((x,y),OTHER['lastMeasurement'])
        vectorDistance = distance_between((x,y),(x+xVector,y+yVector))
        OTHER['distance'] = vectorDistance
        X,P = kalman_filter((x,y),OTHER,X,P)
        OTHER['lastMeasurement'] = (x,y)

        if didCollide:#Reset historical
            historicalDeque = deque(maxlen=historicalDequeSize)
        else:
            normalizedVector = (OTHER['distance']*cos(OTHER['lastAngle'] + OTHER['turningAngle']),OTHER['distance']*sin(OTHER['lastAngle'] + OTHER['turningAngle']))
            #normalizedVector = (cos(OTHER['lastAngle'] + OTHER['turningAngle']),sin(OTHER['lastAngle'] + OTHER['turningAngle']))
            if len(historicalDeque) == historicalDequeSize:
                historicalDeque.popleft()
            historicalDeque.append(normalizedVector)
        if visualize:
            unknown_robot.goto(int(X.value[0][0])/2-500, 300-int(X.value[1][0])/2) # flip the y and offset the x
            unknown_robot.pendown()            
    #time.sleep(30)
    return predictions


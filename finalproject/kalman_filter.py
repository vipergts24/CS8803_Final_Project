from matrix import *
from util import *
import turtle
X = None
P = None
measurement_noise = 0.05

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
        OTHER['lastAngle'] = angle
        OTHER['turningAngle'] = angle - OTHER['lastAngle']
        OTHER['distance'] = distance_between((x,y),OTHER['lastMeasurement'])
        X,P = kalman_filter((x,y),OTHER,X,P)
        OTHER['lastMeasurement'] = (x,y)
        if visualize:
            predict_robot.goto(int(X.value[0][0])/2-500, 300-int(X.value[1][0])/2) # flip the y and offset the x
            predict_robot.pendown() # or .stamp()
    predictions = []
    #Two Second Predictions - Blue
    if visualize:
        unknown_robot = turtle.Turtle()
        unknown_robot.shape('turtle')
        unknown_robot.color('blue')
        unknown_robot.resizemode('user')
        unknown_robot.shapesize(0.25, 0.25, 0.25)
        unknown_robot.penup()
    for i in range(60):
        x = int(X.value[0][0])
        y = int(X.value[1][0])
        predictions.append((x,y))
        angle = atan2(y-OTHER['lastMeasurement'][1],x-OTHER['lastMeasurement'][0])
        angle = angle_trunc(angle)
        OTHER['lastAngle'] = angle
        OTHER['turningAngle'] = angle - OTHER['lastAngle']
        OTHER['distance'] = distance_between((x,y),OTHER['lastMeasurement'])
        X,P = kalman_filter((x,y),OTHER,X,P)
        OTHER['lastMeasurement'] = (x,y)
        if visualize:
            unknown_robot.goto(int(X.value[0][0])/2-500, 300-int(X.value[1][0])/2) # flip the y and offset the x
            unknown_robot.pendown()            
    return predictions


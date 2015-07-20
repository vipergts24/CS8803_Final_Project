# You want to make sure your version produces better error rates than this :)

import sys

filename = "Inputs/test07.txt" #sys.argv[1]
x, y = open(filename, 'r').readlines()[-1].split(',')


#with open('prediction.txt', 'w') as f:
#    for _ in range(60):
#        print >> f, '%s,%s' % (x.strip(), y.strip())


def show_movement():
    import turtle
    window = turtle.Screen()
    window.bgcolor('white')
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.25, 0.25, 0.25)
    broken_robot.penup()
    #End of Visualization
    with open(filename) as f:
        for line in f:
            x, y = line.split(",")
            print x,y
            broken_robot.goto(int(x)/2-500, 300-int(y)/2) # flip the y and offset the x
            broken_robot.pendown() # or .stamp()
    return True

show_movement()

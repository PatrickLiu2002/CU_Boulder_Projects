"""line_following controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")

#Set left/right wheels to turn
leftMotor.setPosition(float('Inf'))
rightMotor.setPosition(float('Inf'))

#Getting devices
gs = []

MAX_SPEED = 6.28

for i in range(3):
    gs.append(robot.getDevice('gs' + str(i)))
    gs[-1].enable(timestep)

#Tracking Variables
totalDist = 0
totalRot = 0
r = 0.0201
t = 32/1000
d = 0.052

#Variables for world position

#xw, yw are position
xw = 0
yw = 0.028

#omegaz is orientation (initialized to 90 degrees)
omegaz = 1.5708

#Flag to check if the robot is actively turning.
#If it is turning, we should NOT stop the robot.
turning = False

#Flag to check if the robot should move
move = True
# Main loop:
while robot.step(timestep) != -1:
    g = []
    for gsensor in gs:
        g.append(gsensor.getValue())
        
    #The robot drives straight. It is not turning.    
    if (g[0] > 500 and g[1] < 350 and g[2] > 500): # drive straight
        phildot, phirdot = MAX_SPEED, MAX_SPEED
        turning = False

    #If the robot is driving straight and not turning and meets hte line
    #Stop it and print the distance
    elif(g[0] < 350 and g[1] < 350 and g[2] < 350 and not turning):
        phildot, phirdot = 0, 0
        move = False

    #Handle turns
    elif(g[2]<550 and g[0] > 350): # turn right
        phildot, phirdot = 0.25 *MAX_SPEED, -0.1 * MAX_SPEED
        #This flag prevents the robot from stopping while turning
        turning = True

    elif(g[0]<550 and g[2] > 350): # turn left
        phildot, phirdot = -0.1 * MAX_SPEED, 0.25 * MAX_SPEED
        turning = True


    #Control movement
    if(move):
        leftMotor.setVelocity(phildot)
        rightMotor.setVelocity(phirdot)
    else:
        #If stopped, set velocity to 0
        phildot, phirdot = 0, 0
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)

    #Odometry. Compute Deltax and change in angle
    #Calculate change in position
    deltax = (r*phildot + r*phirdot)/2 * t
    #calculate change in orientation
    #Note that d has been increased to compensate for inertia
    alpha = (r*phirdot - r*phildot)/(d*1.09) * t
    
    #Calculate coordinates and angle
    #Desired final coordinates are dependent on where it stops
    xw = xw + np.cos(omegaz)*deltax
    yw = yw + np.sin(omegaz)*deltax
    
    #Desired final rotation is something like -270 degrees
    #As the initial angle is 90 degrees
    #And the robot completes a full -360 degree circuit
    
    #Update orientation
    omegaz = omegaz + alpha

    print("xw: ", xw, " yw: ", yw, " omegaz: ", omegaz*180/3.14159)

    pass

# Enter here exit cleanup code.


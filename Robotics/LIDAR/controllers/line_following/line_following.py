"""line_following controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

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

totalDist = 0
totalRot = 0
r = 0.0201
t = 32/1000
d = 0.052

turning = False
move = True
# Main loop:
while robot.step(timestep) != -1:
    g = []
    for gsensor in gs:
        g.append(gsensor.getValue())
        
    #print(g)
    
    if (g[0] > 500 and g[1] < 350 and g[2] > 500): # drive straight
        phildot, phirdot = MAX_SPEED, MAX_SPEED
        turning = False

    elif(g[0] < 350 and g[1] < 350 and g[2] < 350 and not turning):
        phildot, phirdot = 0, 0
        move = False
        print("Dist: " + str(totalDist) + " Rot: " + str(totalRot/3.14159*180))

    elif(g[2]<550 and g[0] > 350): # turn right
        phildot, phirdot = 0.25 *MAX_SPEED, -0.1 * MAX_SPEED
        #This flag prevents the robot from stopping while turning
        turning = True

    elif(g[0]<550 and g[2] > 350): # turn left
        phildot, phirdot = -0.1 * MAX_SPEED, 0.25 * MAX_SPEED
        turning = True



    if(move):
        leftMotor.setVelocity(phildot)
        rightMotor.setVelocity(phirdot)
    else:
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)

    #Odometry
    totalDist += (r*phildot + r*phirdot)/2 * t
    totalRot += (r*phildot - r*phirdot)/d * t
    
    #print("Dist: " + str(totalDist) + " Rot: " + str(totalRot/3.14159*180))


    pass

# Enter here exit cleanup code.

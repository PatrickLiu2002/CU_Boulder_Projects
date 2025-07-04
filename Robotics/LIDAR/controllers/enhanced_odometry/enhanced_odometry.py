"""line_following controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np
from matplotlib import pyplot as plt
from scipy import signal

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")

#Set left/right wheels to turn
leftMotor.setPosition(float('Inf'))
rightMotor.setPosition(float('Inf'))

lidar = robot.getDevice('LDS-01')
lidar.enable(timestep)
lidar.enablePointCloud()

display = robot.getDevice('display')

#Getting Line Following Sensors
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

#GPS and Compass
gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

#Flag to check if the robot is actively turning.
#If it is turning, we should NOT stop the robot.
turning = False

#Flag to check if the robot should move
move = True

#Stuff for mapping
X = 0
angles = np.linspace(3.1415, -3.1415, 360)
map = np.zeros((300, 300))

def world2map(xw, yw):
    #0.305, 0.25 is the center
    #Arena is 1 x 1.
    
    #Adjust to -0.5, -0.5 is now 0, 0
    
    #Coordinate will be adjusted by 0.305, 0.25
    recenX = xw - 0.305 + 0.5
    recenY = yw - 0.25 + 0.5
    
    #Now, 0, 0 is the center of the arena
    
    #Rescale. Multiply by 300. 
    mx = recenX * 299
    my = recenY * 299
    
    #We will convert this to a value between 0 and 300
    
    if(mx > 299):
        mx = 299
    if (mx < 0):
        mx = 0
    if(my > 299):
        my = 299
    if (my < 0):
        my = 0
    
    return int(mx), int(299 - my)

# Main loop:
while robot.step(timestep) != -1:

    xw = gps.getValues()[0]
    yw = gps.getValues()[1]
    theta = np.arctan2(compass.getValues()[0], compass.getValues()[1])

    #Ground Sensoprs
    g = []
    for gsensor in gs:
        g.append(gsensor.getValue())
    
    w_T_r = np.array([[np.cos(theta),-np.sin(theta), xw],
                  [np.sin(theta),np.cos(theta), yw],
                  [0,0,1]])
                              
    ranges = np.array(lidar.getRangeImage())
    ranges[ranges == np.inf] = 100
    
    X_r = np.array([ranges*np.cos(angles), 
                    ranges*np.sin(angles),
                    np.ones(len(angles))])
    
    D = w_T_r @ X_r
    
    #work on garphing D
    for item in np.transpose(D):
        display.setColor(0xFFFFFF)
        px, py = world2map(item[0], item[1])
        map[px, py] += 0.1


        v=int(map[px, py]*255)
        color=(v*256**2+v*256+v)
        if(color > 0xFFFFFF): color = 0xFFFFFF
        if(color < 0): color = 0

        display.setColor(color)
        display.drawPixel(px, py)
    
    
    #The robot drives straight. It is not turning.    
    if (g[0] > 500 and g[1] < 350 and g[2] > 500): # drive straight
        phildot, phirdot = MAX_SPEED, MAX_SPEED
        turning = False

    #If the robot is driving straight and not turning and meets hte line
    elif(g[0] < 350 and g[1] < 350 and g[2] < 350 and not turning):
        phildot, phirdot = 0, 0
        move = False
        
        
        kernel= np.ones((45,45))       

        cmap = signal.convolve2d(map,kernel,mode='same')
        
        cspace = cmap>0.9

        plt.imshow(cspace)
        plt.show()

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



    display.setColor(0xFF0000)
    px, py = world2map(xw, yw)
    display.drawPixel(px, py)
    
    pass

# Enter here exit cleanup code.

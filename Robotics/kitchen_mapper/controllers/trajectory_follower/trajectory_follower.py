"""trajectory chsaing controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Supervisor
import numpy as np
from matplotlib import pyplot as plt
from scipy import signal

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

leftMotor = robot.getDevice("wheel_left_joint")
rightMotor = robot.getDevice("wheel_right_joint")

#Set left/right wheels to turn
leftMotor.setPosition(float('Inf'))
rightMotor.setPosition(float('Inf'))

lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

display = robot.getDevice('display')

MAX_SPEED = 6.28

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
angles = np.linspace(1.57, -1.57, 507)
map = np.zeros((300, 300))

#Ping Pong Ball Controller
marker = robot.getFromDef("marker").getField("translation")
marker.setSFVec3f([0, 0, 0.2])

#Set up waypoints
WP = [(0.63, -3.29), (-1.58,-2.72),
(-1.64, 0.36), (0.63, 0.36), (-1.52, 0.36),
 (-1.68,-2.85),(0.63, -3.29),(0.63, -0.45),
 (0.63, 0.45), (0,0)] #Last is placeholder

index = 0

def world2map(xw, yw):

    #Adjust center
    recenX = xw + 0.65 + 2
    recenY = yw + 1.43 + 2.5
    
    #Now, 0, 0 is the center of the arena
    
    #Rescale. Multiply by 300. 
    mx = recenX/5 * 299
    my = recenY/5 * 299
    
    #We will convert this to a value between 0 and 300
    
    if(mx > 299):
        mx = 299
    if (mx < 0):
        mx = 0
    if(my > 299):
        my = 299
    if (my < 0):
        my = 0
    
    return int(299 - mx), int(my)

# Main loop:
while robot.step(timestep) != -1:
    xw = gps.getValues()[0]
    yw = gps.getValues()[1]
    theta = np.arctan2(compass.getValues()[0], compass.getValues()[1])

    #Waypoint set
    marker.setSFVec3f([*WP[index],0])

    #Find Error
    rho = np.sqrt((xw - WP[index][0])**2 + (yw - WP[index][1])**2)
    alpha = np.arctan2(WP[index][1] - yw, WP[index][0] - xw) - theta
    absErr = np.arctan2(WP[index][1] - yw, WP[index][0] - xw)

    if(alpha > np.pi):
        alpha = alpha - 2 * np.pi
    if(alpha < -np.pi):
        alpha = alpha + 2 * np.pi
    
    if(rho < 0.3):
        index = index + 1
        marker.setSFVec3f([*WP[index],0])
    
    p1, p2 = 5, 2

    
    phildot = -alpha * p1 + rho * p2
    phirdot = alpha * p1 + rho * p2


    if(phildot > MAX_SPEED):
        phildot = MAX_SPEED
    if(phirdot > MAX_SPEED):
        phirdot = MAX_SPEED

    if(phildot < -MAX_SPEED):
        phildot = -MAX_SPEED
    if(phirdot < -MAX_SPEED):
        phirdot = -MAX_SPEED



    leftMotor.setVelocity(phildot)
    rightMotor.setVelocity(phirdot)

    if(index > 8):
        phildot = 0
        phirdot = 0
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        print("Path following complete")
        #36 yields very small path
        kernel= np.ones((36,36))       

        cmap = signal.convolve2d(map,kernel,mode='same')
        
        cspace = cmap>0.9

        plt.imshow(cspace)
        plt.show()

    w_T_r = np.array([[np.cos(theta),-np.sin(theta), xw],
                  [np.sin(theta),np.cos(theta), yw],
                  [0,0,1]])
                              
    ranges = np.array(lidar.getRangeImage())[80:587]
    ranges[ranges == np.inf] = 1000
    
    
    X_r = np.array([ranges*np.cos(angles), 
                    ranges*np.sin(angles),
                    np.ones(len(angles))])
    
    D = w_T_r @ X_r
    
    for item in np.transpose(D):
        display.setColor(0xFFFFFF)
        
        #Adjust for cam position
        offX = 0.2 * np.cos(theta)
        offY = 0.2 * np.sin(theta)
        px, py = world2map(item[0] + offX, item[1] + offY)
        map[px, py] += 0.1


        v=int(map[px, py]*255)
        color=(v*256**2+v*256+v)
        if(color > 0xFFFFFF): color = 0xFFFFFF
        if(color < 0): color = 0

        display.setColor(color)
        display.drawPixel(px, py)


    display.setColor(0xFF0000)
    
    #Adjust for the LIDAR's position
    px, py = world2map(xw, yw)
    display.drawPixel(px, py)

    pass

# Enter here exit cleanup code.

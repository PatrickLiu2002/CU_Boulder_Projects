#Utility module

#Import robot items and numpy
from controller import Robot, Supervisor
import numpy as np

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#Get left/right motors and set them to turn
leftMotor = robot.getDevice("wheel_left_joint")
rightMotor = robot.getDevice("wheel_right_joint")
leftMotor.setPosition(float('Inf'))
rightMotor.setPosition(float('Inf'))

#Set max speed of wheels
MAX_SPEED = 6.28

#Set LIDAR
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

#Set Display
display = robot.getDevice('display')

#GPS and Compass initialization
gps = robot.getDevice('gps')
gps.enable(timestep)
compass = robot.getDevice('compass')
compass.enable(timestep)

#Ping Pong Ball Controller
marker = robot.getFromDef("marker").getField("translation")
marker.setSFVec3f([0, 0, 0.2])

#Define Blackboard class
class blackboard:
    def __init__(self):
        self.data = {
        #The table of coordinates for moving around the table
        'TableWP': [(0.63, -3.29), (-1.58,-2.72),
    (-1.64, 0.36), (0.63, 0.36), (-1.52, 0.36),
     (-1.68,-2.85),(0.63, -3.29),(0.63, -0.45),
     (0.63, 0.45), (0,0)], #Last is placeholder
         
         #Tools to draw a map.
         #Includes angles for the LIDAR and the default empty map        
         'map': np.zeros((200, 300)),
         'angles': np.linspace(1.57, -1.57, 507)

        }

    #Method to write to the blackboard
    def write(self, key, value):
        self.data[key] = value

    #Method to read from the blackboard
    def read(self, key):
        return self.data.get(key)

#A utility function to handle translating world to map coords
def world2map(xw, yw):

    #Adjust center
    recenX = xw + 0.65 + 2
    recenY = yw + 1.43 + 2.5
    
    #Now, 0, 0 is the center of the arena
    
    #Rescale. Multiply by 300. 
    mx = recenX/5 * 199
    my = recenY/5 * 299
    
    #We will convert this to a value between 0 and 300
    
    if(mx > 199):
        mx = 199
    if (mx < 0):
        mx = 0
    if(my > 299):
        my = 299
    if (my < 0):
        my = 0
    
    return int(199 - mx), int(my)

#Reverses world2map, turning map coords into world coords
#The logical result of using algebra on world2map
def map2world(xm, ym):

    nuX = 199 - xm
    nuY = ym
    
    nuX = nuX * 5 / 199
    nuY = nuY * 5 / 299
    
    return nuX - 0.65 - 2, nuY - 1.43 - 2.5
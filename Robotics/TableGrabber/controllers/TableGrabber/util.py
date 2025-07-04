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
MAX_SPEED = 10

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


camera = robot.getDevice("Astra rgb")
camera.enable(timestep)
camera.recognitionEnable(timestep)

#Commission all joints and snsors
robot_joints = {
       'torso_lift_joint' : 0.35,
       'arm_1_joint' : 0.71,
       'arm_2_joint' : 1.02,
       'arm_3_joint' : -2.815,
       'arm_4_joint' : 1.011,
       'arm_5_joint' : 0,
       'arm_6_joint' : 0,
       'arm_7_joint' : 0,
       'gripper_left_finger_joint' : 0,
       'gripper_right_finger_joint': 0,
       'head_1_joint':0,
       'head_2_joint':0
}

robot_sensors = [
       'torso_lift_joint_sensor',
       'arm_1_joint_sensor',
       'arm_2_joint_sensor',
       'arm_3_joint_sensor',
       'arm_4_joint_sensor',
       'arm_5_joint_sensor',
       'arm_6_joint_sensor',
       'arm_7_joint_sensor',
       'gripper_left_sensor_finger_joint',
       'gripper_right_sensor_finger_joint',
       'head_1_joint_sensor',
       'head_2_joint_sensor'
]


#gripper_right_sensor_finger_joint

encoders={}
for jname in robot_sensors:
    sname=jname
    encoders[jname]= robot.getDevice(sname)
    encoders[jname].enable(timestep)


robot.getDevice('gripper_left_finger_joint').enableForceFeedback(timestep)
robot.getDevice('gripper_right_finger_joint').enableForceFeedback(timestep)

#Define Blackboard class
class blackboard:
    def __init__(self):
        self.data = {
        #The table of coordinates for moving around the table
        'TableWP': [(0, 0), (0.39, -2.88), (-1.58,-2.72),(-1.64, -2.5),
    (-1.64, 0.36), (0.86, 0.40), (0.63, 0.36), (-1.52, 0.36),(-1.7, -0.45), 
     (-1.56, -2.26), (-1.30,-2.85),(0.21, -2.8),(0.63, -0.45),
     (0.6, -0.3), (1.36, 0.12), (0,0)], #Last is placeholder
         
         #Tools to draw a map.
         #Includes angles for the LIDAR and the default empty map        
         'map': np.zeros((200, 300)),
         'angles': np.linspace(1.57, -1.57, 507),
         
         #Coordinates used to orient the robot
         #TableCenter is the center of the table
         'tableCenter': (-0.65, -1.43),
         
         #outside is a generic point that points the bot
         #to the left of the counter, allowing it to scan
         'outside': (2.01, 1.48),
          'back': (-2.25, -0.45),
         
         #The color IDs of desired objects.
         'jam': [0.55, 0.06, 0.06],
         'honey': [1, 0.57, 0]
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
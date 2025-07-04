#Utility module

#Import robot items and numpy
from controller import Robot, Supervisor
import numpy as np
import math

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
marker.setSFVec3f([1.13, -0.34, 0.2])

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
    encoders[jname]=robot.getDevice(sname)
    encoders[jname].enable(timestep)

def isAcceptable(a, b):
    for x in range(len(a)):
        if(abs(a[x] - b[x]) > 0.001):
            return False
    return True

def setArms():
    robot.getDevice('arm_1_joint').setPosition(math.pi/2)
    robot.getDevice('arm_2_joint').setPosition(math.pi/4)
    robot.getDevice('arm_4_joint').setPosition(math.pi/4)
    robot.getDevice('arm_5_joint').setPosition(math.pi/2)
    robot.getDevice('gripper_left_finger_joint').setPosition(0.045)
    robot.getDevice('gripper_right_finger_joint').setPosition(0.045)
    robot.getDevice('torso_lift_joint').setPosition(0.05)

    desiredPos = [math.pi/2, math.pi/4, math.pi/4, math.pi/2, 0.045, 0.045]
    currentPos = [0, 0, 0, 0, 0, 0]
    
    
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
        
    
    while(not isAcceptable(desiredPos, currentPos)):
        currentPos[0] = encoders['arm_1_joint_sensor'].getValue()
        currentPos[1] = encoders['arm_2_joint_sensor'].getValue()
        currentPos[2] = encoders['arm_4_joint_sensor'].getValue()
        currentPos[3] = encoders['arm_5_joint_sensor'].getValue()
        currentPos[4] = encoders['gripper_left_sensor_finger_joint'].getValue()
        currentPos[5] = encoders['gripper_right_sensor_finger_joint'].getValue()
        robot.step(timestep)
    return True
        
setArms()

JC = { 'openGripper' : {'gripper_left_finger_joint' : 0.045,
                        'gripper_right_finger_joint': 0.045},
       'closeGripper': {'gripper_left_finger_joint' : 0.0,
                        'gripper_right_finger_joint': 0.0},
       'armReady': {'arm_1_joint': math.pi/2,
       'arm_2_joint': math.pi/4,
       'arm_4_joint': math.pi/4,
       'arm_5_joint': math.pi/2,} }

# Main loop:
#Get an object from a list of objects based on its color
def getObject(objects, colors):
    for object in objects:
        if(object.getColors()[0] == colors[0] and object.getColors()[1] == colors[1] and object.getColors()[2] == colors[2]):
            return object
    return None

def rotatePoint(WP):
    #Aligns the robot with a point rotationally
    #IE. This lets the robot point itself at an object
    #Or a waypoint if you want.
    
    #It's here to let the robot do little things like
    #Dropping a jar onto the table w/o dropping it off the table
    
    #And also for generally pointing towards where something is

    alpha = np.pi
    
    while (abs(alpha) > 0.005):
        robot.step(timestep)

        xw = gps.getValues()[0]
        yw = gps.getValues()[1]
        
        theta = np.arctan2(compass.getValues()[0], compass.getValues()[1])
        
        alpha = np.arctan2(WP[1] - yw, WP[0] - xw) - theta
        if(alpha > np.pi):
            alpha = alpha - 2 * np.pi
        if(alpha < -np.pi):
            alpha = alpha + 2 * np.pi
        
        
        p1 = 4
        
        phildot = -alpha * p1
        phirdot = alpha * p1  
        
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
        
    phildot = 0
    phirdot = 0
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    return True

#rotatePoint((1.71, -0.302))    
#rotatePoint((1.93, 0.494))    

#Grabs an object using colors and an initial point
#Initial point just needs to put the object in the robot's field of view

#This requires the robot to be in a position where it can approach the object
#Without fear of collision with other things
def rotateCam(wantedColor):
    err = float('Inf')

    while(abs(err + 0.0435) > 0.001):
        robot.step(timestep)
        objects=camera.getRecognitionObjects()
        item = getObject(objects, wantedColor)
        if(item == None):
            print("Item not seen, scanning")
            phildot = 1
            phirdot = -2*math.pi
    
            leftMotor.setVelocity(phildot)
            rightMotor.setVelocity(phirdot) 
            continue
        
        pos = list(item.getPosition())
        
        if(pos[0] <= 0.95):
        
            print("Item too close, reversing")
            phildot = -2
            phirdot = -2
            leftMotor.setVelocity(phildot)
            rightMotor.setVelocity(phirdot) 
            continue
        
        err = pos[1]
        
        p1 = 1
        phildot = -err * p1
        phirdot = err * p1  
        print("pos", pos)

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
        
               
    phildot = 0
    phirdot = 0
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)      
    print("DONE")
    return True

def grabObject(wantedColor):
    offDist = float('Inf')
    #Rotate to match current item
    rotateCam(wantedColor)
    
    
    #move on until item is within 0.787m from the robot
    
    while(offDist > 0.787):
        robot.step(timestep)

        objects=camera.getRecognitionObjects()
        phildot = 1
        phirdot = 1
        leftMotor.setVelocity(phildot)
        rightMotor.setVelocity(phirdot) 
    
    
        wantedColor = [0.55, 0.06, 0.06]
        item = getObject(objects, wantedColor)
        offDist = list(item.getPosition())[0]
    phildot = 0
    phirdot = 0
    leftMotor.setVelocity(phildot)
    rightMotor.setVelocity(phirdot)
    
    #gripItem()
    
    #liftItem()         
        
    return 0

wantedColor = [0.55, 0.06, 0.06]
grabObject(wantedColor)

    #Move forwards 
    
    
    #Grasp until force feedback returns properly
    
    
    #Raise robot torso joint
            
"""
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    objects=camera.getRecognitionObjects()
    
    wantedColor = [0.55, 0.06, 0.06]
    item = getObject(objects, wantedColor)
    
    
    
    #print(item.getColors()[0], item.getColors()[1], item.getColors()[2])
    #print(list(item.getPosition())) 
    #for object in objects:
    #    print(object.getColors()[0], object.getColors()[1], object.getColors()[2])
    #    print(list(object.getPosition())) 
    
       
    #Center object so that 
    
    

    #print(encoders['arm_1_joint_sensor'].getValue(), encoders['torso_lift_joint_sensor'].getValue())
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass
    


# Enter here exit cleanup code.
"""

#Calculate acceptable joint range.
import math
import Tree
import util
import numpy as np

#A function that checks that two lists are within a margin of each other.
#Useful for robot arm movement.
def isAcceptable(a, b):
    for x in range(len(a)):
        if(abs(a[x] - b[x]) > 0.001):
            return False
    return True

#prepares arm for grabbing
class unstow(Tree.Node):

    def __init__ (self, name = None):

        self.name = name or self.__class__.__name__

    #Unstow arm in stages so it's ready to grag
    #Note that joints are adjusted in groups to avoid clipping
    def execute(self): 
        print("Unstowing arm")    
        util.leftMotor.setVelocity(0)
        util.rightMotor.setVelocity(0)
        currentPos = [0, 0, 0, 0]
    
        util.robot.getDevice('arm_2_joint').setPosition(math.pi/4)
        desiredPos = [0, math.pi/4, 0, 0]
        while(not isAcceptable(desiredPos, currentPos)):
            currentPos[1] = util.encoders['arm_2_joint_sensor'].getValue()
            util.robot.step(util.timestep)
    
        util.robot.getDevice('arm_1_joint').setPosition(math.pi/2)
        util.robot.getDevice('arm_4_joint').setPosition(math.pi/4)
        util.robot.getDevice('arm_5_joint').setPosition(math.pi/2)
        util.robot.getDevice('torso_lift_joint').setPosition(0.05)
        util.robot.getDevice('arm_3_joint').setPosition(0)
        util.robot.getDevice('arm_6_joint').setPosition(0)

        desiredPos = [math.pi/2, math.pi/4, math.pi/4, math.pi/2]
        
        while(not isAcceptable(desiredPos, currentPos)):
            currentPos[0] = util.encoders['arm_1_joint_sensor'].getValue()
            currentPos[2] = util.encoders['arm_4_joint_sensor'].getValue()
            currentPos[3] = util.encoders['arm_5_joint_sensor'].getValue()
            util.robot.step(util.timestep)

        return True
        
#Releases grippers.
class release(Tree.Node):

    def __init__ (self, name = None):
        self.name = name or self.__class__.__name__

    #Releases the grippers. 
    def execute(self): 
        print("Releasing grippers")
        
        #Stop robot    
        util.leftMotor.setVelocity(0)
        util.rightMotor.setVelocity(0)
        
        #Release grippers
        currentPos = [0, 0]
    
        util.robot.getDevice('gripper_left_finger_joint').setPosition(0.045)
        util.robot.getDevice('gripper_right_finger_joint').setPosition(0.045)

        desiredPos = [0.045, 0.045]
        
        while(not isAcceptable(desiredPos, currentPos)):
            currentPos[0] = util.encoders['gripper_left_sensor_finger_joint'].getValue()
            currentPos[1] = util.encoders['gripper_right_sensor_finger_joint'].getValue()
            util.robot.step(util.timestep)

        return True

#Stores arm. Does this in stages to avoid clipping.
class stow(Tree.Node):

    def __init__ (self, name = None):

        self.name = name or self.__class__.__name__

    #Dummy execution method
    def execute(self): 
        print("Stowing arm")
        util.leftMotor.setVelocity(0)
        util.rightMotor.setVelocity(0)
        util.robot.getDevice('arm_1_joint').setPosition(math.pi/2)
        
        
        currentPos = [0, 0, 0, 0, 0]
        
        currentPos[0] = util.encoders['arm_1_joint_sensor'].getValue()

        desiredPos = [math.pi/2, 0, 0, 0, 0]
        while(not isAcceptable(desiredPos, currentPos)):
            currentPos[0] = util.encoders['arm_1_joint_sensor'].getValue()

            util.robot.step(util.timestep)
        

        util.robot.getDevice('arm_3_joint').setPosition(1.5)
        util.robot.getDevice('arm_4_joint').setPosition(math.pi/2)
        
        util.robot.getDevice('arm_5_joint').setPosition(0)

        util.robot.getDevice('arm_6_joint').setPosition(1.39)

        desiredPos = [math.pi/2, 0, 1.5, math.pi/2, 1.39]


        while(not isAcceptable(desiredPos, currentPos)):
            currentPos[0] = util.encoders['arm_1_joint_sensor'].getValue()
            currentPos[2] = util.encoders['arm_3_joint_sensor'].getValue()
            currentPos[3] = util.encoders['arm_4_joint_sensor'].getValue()
            currentPos[4] = util.encoders['arm_6_joint_sensor'].getValue()
            util.robot.step(util.timestep)

        desiredPos = [math.pi/2, -1.5, 1.5, math.pi/2, 1.39]

        util.robot.getDevice('arm_2_joint').setPosition(-1.5)
        while(not isAcceptable(desiredPos, currentPos)):
            currentPos[0] = util.encoders['arm_1_joint_sensor'].getValue()
            currentPos[1] = util.encoders['arm_2_joint_sensor'].getValue()
            currentPos[2] = util.encoders['arm_3_joint_sensor'].getValue()
            currentPos[3] = util.encoders['arm_4_joint_sensor'].getValue()
            currentPos[4] = util.encoders['arm_6_joint_sensor'].getValue()
            util.robot.step(util.timestep)
    
        util.leftMotor.setVelocity(0)
        util.rightMotor.setVelocity(0)
            

        while(not isAcceptable(desiredPos, currentPos)):
            currentPos[0] = util.encoders['arm_1_joint_sensor'].getValue()
            currentPos[1] = util.encoders['arm_2_joint_sensor'].getValue()
            currentPos[2] = util.encoders['arm_3_joint_sensor'].getValue()
            currentPos[3] = util.encoders['arm_4_joint_sensor'].getValue()
            currentPos[4] = util.encoders['arm_6_joint_sensor'].getValue()
            util.robot.step(util.timestep)
        return True

#Points towards a certain point. Useful for safing the arm.
class orient(Tree.Node):

    def __init__ (self, key = 'tableCenter', blackboard = None):
        #initialize name, blackboard, completion status, and key
        self.blackboard = blackboard
        self.complete = False
        #The key used to get the orientation point
        self.key = key
    
    def execute(self):
        print("Orienting towards", self.key)
    
        WP = self.blackboard.read(self.key) 

        #Aligns the robot with a point rotationally
        #IE. This lets the robot point itself at an object
        #Or a waypoint if you want.
        
        #It's here to let the robot do little things like
        #Dropping a jar onto the table w/o dropping it off the table
        
        #And also for generally pointing towards where something is
        
        alpha = np.pi
        
        while (abs(alpha) > 0.005):
            util.robot.step(util.timestep)
        
            xw = util.gps.getValues()[0]
            yw = util.gps.getValues()[1]
            
            theta = np.arctan2(util.compass.getValues()[0], util.compass.getValues()[1])
            
            alpha = np.arctan2(WP[1] - yw, WP[0] - xw) - theta
            if(alpha > np.pi):
                alpha = alpha - 2 * np.pi
            if(alpha < -np.pi):
                alpha = alpha + 2 * np.pi
            
            
            p1 = 4
            
            phildot = -alpha * p1
            phirdot = alpha * p1
            #print(phildot, phirdot)
            
            if(phildot > util.MAX_SPEED):
                phildot = util.MAX_SPEED
            if(phirdot > util.MAX_SPEED):
                phirdot = util.MAX_SPEED
        
            if(phildot < -util.MAX_SPEED):
                phildot = -util.MAX_SPEED
            if(phirdot < -util.MAX_SPEED):
                phirdot = -util.MAX_SPEED
        
            util.leftMotor.setVelocity(phildot)
            util.rightMotor.setVelocity(phirdot)
            
        phildot = 0
        phirdot = 0
        util.leftMotor.setVelocity(0)
        util.rightMotor.setVelocity(0)
        print("Finished orienting towards", self.key)

        return True

#A utility filter function that gets an object out of many
#If multiple have the same color, it will return the first
def getObject(objects, colors):
    for object in objects:
        if(object.getColors()[0] == colors[0] and object.getColors()[1] == colors[1] and object.getColors()[2] == colors[2]):
            return object
    return None

#Utility function to rotate the camera towards a given object

#Scans clockwise if it can't see the object
def rotateCam(wantedColor):
    err = float('Inf')
    print("Starting targeting")

    while(abs(err) > 0.01):
        util.robot.step(util.timestep)
        objects = util.camera.getRecognitionObjects()
        item = getObject(objects, wantedColor)
        
        #If there is no item, "Scan" by rotating clockwise.
        #Note that this assumes that the item is close enough to see by rotating.
        if(item == None):
            #print("Item not seen, scanning")
            phildot = 1
            phirdot = -2*math.pi
    
            util.leftMotor.setVelocity(phildot)
            util.rightMotor.setVelocity(phirdot) 
            continue
        
        pos = list(item.getPosition())
        
        if(pos[0] <= 0.95):
        
            #If it's too close for the arm, reverse the bot
            #print("Item too close, reversing")
            phildot = -2
            phirdot = -2
            util.leftMotor.setVelocity(phildot)
            util.rightMotor.setVelocity(phirdot) 
            continue
        #As the arm's center and camera center are off, adjust error
        err = pos[1] + 0.0435
        
        p1 = 1
        phildot = -err * p1
        phirdot = err * p1  
        #print("pos", pos)

        if(phildot > util.MAX_SPEED):
            phildot = util.MAX_SPEED
        if(phirdot > util.MAX_SPEED):
            phirdot = util.MAX_SPEED
    
        if(phildot < -util.MAX_SPEED):
            phildot = -util.MAX_SPEED
        if(phirdot < -util.MAX_SPEED):
            phirdot = -util.MAX_SPEED 
        util.leftMotor.setVelocity(phildot)
        util.rightMotor.setVelocity(phirdot) 


               
    phildot = 0
    phirdot = 0
    util.leftMotor.setVelocity(0)
    util.rightMotor.setVelocity(0)

          
    print("Completed initial targeting of item")
    return True

def adjustCam(wantedColor):
    err = float('Inf')
    #print("Adjusting targeting")

    while(abs(err) > 0.01):
    
        #Similar to rotateCam, but without the scanner or reverse
        #This assumes that the object is within the camera's FoV
        util.robot.step(util.timestep)
        objects = util.camera.getRecognitionObjects()
        item = getObject(objects, wantedColor)
        
        pos = list(item.getPosition())
        
        #As the arm's center and camera center are off, adjust error
        err = pos[1] + 0.0435
        
        p1 = 4
        phildot = -err * p1
        phirdot = err * p1  
        #print("pos", pos)


        #Max speed limiting.
        if(phildot > util.MAX_SPEED):
            phildot = util.MAX_SPEED
        if(phirdot > util.MAX_SPEED):
            phirdot = util.MAX_SPEED
    
        if(phildot < -util.MAX_SPEED):
            phildot = -util.MAX_SPEED
        if(phirdot < -util.MAX_SPEED):
            phirdot = -util.MAX_SPEED 
        util.leftMotor.setVelocity(phildot)
        util.rightMotor.setVelocity(phirdot) 


    #Stop the robot           
    phildot = 0
    phirdot = 0
    util.leftMotor.setVelocity(0)
    util.rightMotor.setVelocity(0)

          
    #print("DONE")
    return True



#Grabs an item.
class grab(Tree.Node):

    #Here, "key" links to a list that matches the color id
    def __init__ (self, key = 'Object1', blackboard = None):
        #initialize name, blackboard, completion status, and key
        self.blackboard = blackboard
        self.complete = False
        #The key used to get the color
        self.key = key
    
    def execute(self):
        print("Getting item", self.key)
        #Gets the desired color of the item
        wantedColor = self.blackboard.read(self.key)
        offDist = float('Inf')
        #Rotate to match current item
        rotateCam(wantedColor)
        
        
        #move on until item is within 0.787m from the robot
        
        while(offDist > 0.787):
            util.robot.step(util.timestep)
            
            objects=util.camera.getRecognitionObjects()

            item = getObject(objects, wantedColor)
            offDist = list(item.getPosition())[0]
            pos = list(item.getPosition())
            err = pos[1] + 0.0435

            phildot = 1
            phirdot = 1
            util.leftMotor.setVelocity(phildot)
            util.rightMotor.setVelocity(phirdot) 
            
            #If drift occurs, adjust camera.
            if(abs(err) > 0.01):
                adjustCam(wantedColor)

        
        #Stop the robot
        phildot = 0
        phirdot = 0
        util.leftMotor.setVelocity(phildot)
        util.rightMotor.setVelocity(phirdot)
        
        #Grip, lift, and reverse.
        gripItem()
        
        liftItem()  
        
        reverse()       
        print("Item retrieved")    
        return True    


#Reactive gripping of items.
def gripItem():
    #Close arms until force > certain value
    str1 = 0
    str2 = 0
    util.robot.getDevice('gripper_left_finger_joint').setPosition(0)
    util.robot.getDevice('gripper_right_finger_joint').setPosition(0)
    MAX = 0.045
    MIN = 0
    
    while(str1 > -10 and str2 > -10):
        util.robot.step(util.timestep)
        str1 = util.robot.getDevice('gripper_left_finger_joint').getForceFeedback()
        str2 = util.robot.getDevice('gripper_right_finger_joint').getForceFeedback()
        pos1 = util.encoders['gripper_left_sensor_finger_joint'].getValue()
        pos2 = util.encoders['gripper_right_sensor_finger_joint'].getValue()

    #Set motor position to where they are now
    
    
    #Because of WeBots, sometimes the arm can be
    #Set to a position greater than max. This avoids errors from that.
    if(pos1 > MAX):
        pos1 = MAX
    if(pos2 > MAX): 
        pos2 = MAX
    if(pos1 < MIN):
        pos1 = MIN
    if(pos2 < MIN): 
        pos2 = MIN
    
    #Set grippers to their current position. 
    util.robot.getDevice('gripper_left_finger_joint').setPosition(pos1)
    util.robot.getDevice('gripper_right_finger_joint').setPosition(pos2)


    #return True.
    return True

#Utility function to lift the arm.
def liftItem():
    #Uses the elbow joint to lift an item
    #Does not control grippers. This is also used
    #To clear the arm of obstacles like the chairs.
    util.robot.getDevice('arm_4_joint').setPosition(0)
    while(abs(util.encoders['arm_4_joint_sensor'].getValue()) > 0.01):
        util.robot.step(util.timestep)

#Lifts arm manually. Usually for clearing obstacles.
class lift(Tree.Node):

    def __init__ (self, name):
        self.name = name
        #initialize name, blackboard, completion status, and key

    
    def execute(self):
        print("Lifting arm")
        liftItem()
        return True

#Utility function to reverse the robot to give it more space
def reverse():
    util.leftMotor.setVelocity(-6)
    util.rightMotor.setVelocity(-6) 
    
    for x in range(40):
        util.robot.step(util.timestep)

    util.leftMotor.setVelocity(0)
    util.rightMotor.setVelocity(0) 

#Unused manual reverse.
class reverseBot(Tree.Node):

    #Here, "key" links to a list that matches the color id
    def __init__ (self, name):
        self.name = name
        #initialize name, blackboard, completion status, and key

    
    def execute(self):
        print("Reversing robot")
        reverse()
        return True
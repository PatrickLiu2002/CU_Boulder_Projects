#Given a list of waypoints, it will drive to the points
import util
import Tree
import numpy as np

#The "drive robot" node that follows waypoints.
#Drives robot according to a set of waypoints
#Default behavior is driving around the table.
class driveRobotWP(Tree.Node):

    def __init__ (self, name = None, blackboard = None, key = 'TableWP'):
        #initialize name, blackboard, completion status, and key
        self.blackboard = blackboard
        self.name = name or self.__class__.__name__
        self.complete = False
        #The key used to get the waypoints
        #Note that this is not the name
        self.key = key

    #Dummy execution method
    def execute(self): 
        #Set up robot and perform timestep
        self.setup()
        
        #While not complete, timestep and tick
        while(not self.complete):
            util.robot.step(util.timestep)
            self.tick()
            
        #Terminate and return
        self.terminate()
        return True

    #Setup method
    def setup(self):
        #Print and initialize waypoints + index
        print("Setting up ", self.name)
        self.WP = self.blackboard.read(self.key) #Last is placeholder
        self.index = 0
        #Return
        return "RUNNING"

    #Tick method
    def tick(self):

        #Get position and angle
        xw = util.gps.getValues()[0]
        yw = util.gps.getValues()[1]
        theta = np.arctan2(util.compass.getValues()[0], util.compass.getValues()[1])
        #Waypoint set
        util.marker.setSFVec3f([*self.WP[self.index],0])
    
        #Find Error
        rho = np.sqrt((xw - self.WP[self.index][0])**2 + (yw - self.WP[self.index][1])**2)
        alpha = np.arctan2(self.WP[self.index][1] - yw, self.WP[self.index][0] - xw) - theta
        absErr = np.arctan2(self.WP[self.index][1] - yw, self.WP[self.index][0] - xw)

        #Arctan2 adjustment    
        if(alpha > np.pi):
            alpha = alpha - 2 * np.pi
        if(alpha < -np.pi):
            alpha = alpha + 2 * np.pi

        #Increment waypoints        
        if(rho < 0.3):
            self.index = self.index + 1
            util.marker.setSFVec3f([*self.WP[self.index],0])

        #Set multipliers for behavior        
        p1, p2 = 5, 3
    
        #Set wheel speeds        
        phildot = -alpha * p1 + rho * p2
        phirdot = alpha * p1 + rho * p2
    
        #Set phildot, phirdot to max speed if beyond limits.

        if(phildot > util.MAX_SPEED):
            phildot = util.MAX_SPEED
        if(phirdot > util.MAX_SPEED):
            phirdot = util.MAX_SPEED
    
        if(phildot < -util.MAX_SPEED):
            phildot = -util.MAX_SPEED
        if(phirdot < -util.MAX_SPEED):
            phirdot = -util.MAX_SPEED
        
        #Set motor speeds
        util.leftMotor.setVelocity(phildot)
        util.rightMotor.setVelocity(phirdot)
    
        #If we're at the last waypoint (always a dummy)
        #We stop things and terminate
        if(self.index == len(self.WP)-1):
            self.complete = True
            return True

        #Return        
        return "RUNNING"
        
    #Ends driving
    def terminate(self):
    
        #Stop all wheels and return True.
        phildot = 0
        phirdot = 0
        util.leftMotor.setVelocity(0)
        util.rightMotor.setVelocity(0)
        print("Path following complete")        
    
        #Successful process.
        return True        
            
        
    
    
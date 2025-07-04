#Code to draw maps 
#Import necessary items
import util
import Tree
import numpy as np
from matplotlib import pyplot as plt
from scipy import signal
from os.path import exists

#Tests for a map's existence
class TestForMap(Tree.Node):

    #Execution
    def execute(self):
        print("Executing action:", self.name)
        
        #Check for cspace.npy and return True if it's here
        file_exists = exists('cspace.npy')
        print("File Exists: " + str(file_exists))
        return file_exists
        
#Draws a map.
class drawMap(Tree.Node):

    #Initialize with blackboard.
    def __init__ (self, name = None, blackboard = None):
        self.blackboard = blackboard
        self.name = name or self.__class__.__name__

    #Dummy execution method. Not useful at all.
    def execute(self):
        #Timestep, setup, tick, and finally terminate.
        util.robot.step(util.timestep)
        self.setup()
        print("Warning: drawMap should not be executed, use as a parallel process instead.")
        self.tick()
        self.terminate()

    #Setup with map and angles.
    def setup(self):
        print("Starting mapping")
        self.map = self.blackboard.read('map') #Last is placeholder
        self.angles = self.blackboard.read('angles') #Last is placeholder
        return "RUNNING"

    #Tick        
    def tick(self):
        #Get world coords and angle from gps
        xw = util.gps.getValues()[0]
        yw = util.gps.getValues()[1]
        theta = np.arctan2(util.compass.getValues()[0], util.compass.getValues()[1])
    
        
        #Get transform array 
        w_T_r = np.array([[np.cos(theta),-np.sin(theta), xw],
                      [np.sin(theta),np.cos(theta), yw],
                      [0,0,1]])
        
        #Get ranges                          
        ranges = np.array(util.lidar.getRangeImage())[80:587]
        ranges[ranges == np.inf] = 1000
        
        #Get array
        X_r = np.array([ranges*np.cos(self.angles), 
                        ranges*np.sin(self.angles),
                        np.ones(len(self.angles))])
        
        #Calculate 
        D = w_T_r @ X_r
        
        #For each reading:
        for item in np.transpose(D):
        
            #Set display color
            util.display.setColor(0xFFFFFF)
            
            #Adjust for LIDAR position
            offX = 0.2 * np.cos(theta)
            offY = 0.2 * np.sin(theta)
            px, py = util.world2map(item[0] + offX, item[1] + offY)
            
            #Update map
            self.map[px, py] += 0.1
            
            #Update blackboard
            self.blackboard.write('map', self.map)
    
            #Set color and draw obstacles
            v=int(self.map[px, py]*255)
            color=(v*256**2+v*256+v)
            if(color > 0xFFFFFF): color = 0xFFFFFF
            if(color < 0): color = 0
            
            util.display.setColor(color)
            util.display.drawPixel(px, py)
    
        #Set color for drawing robot path
        util.display.setColor(0xFF0000)
        
        #Draw robot path
        px, py = util.world2map(xw, yw)
        util.display.drawPixel(px, py)
        
        return "RUNNING"                

    #Once instructed to terminate, saves map after convolving
    def terminate(self):
        #New kernel for updated system
        kernel= np.ones((25,40))       

        #Generate cmap and cspace
        cmap = signal.convolve2d(self.map,kernel,mode='same')
        
        cspace = cmap > 0.9
        self.blackboard.write('map', self.map)

        #Save file
        np.save('cspace',cspace)
        print("Map saved as cspace.npy")
        
        #Return True
        return True

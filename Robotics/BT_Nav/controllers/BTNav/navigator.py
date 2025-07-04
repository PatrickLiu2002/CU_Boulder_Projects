#Imports for below methods
import numpy as np
from skimage.draw import line_nd, random_shapes
from matplotlib import pyplot as plt
from scipy import signal
from os.path import exists
from heapq import heapify, heappush, heappop
from collections import defaultdict
import Tree
import util

#Generates a series of waypoints from current location to.
#a desired destination.  
#Actually a bunch of methods
#Hiding in each other, like three smaller items in a coat
#Pretending to be a big one
class generateWaypoints(Tree.Node):
    
    #Initialize based on name, blackboard, and target coordinates.
    def __init__ (self, name = None, blackboard = None, locx = 0, locy = 0):
        
        #Set blackboard and target coordinates
        self.blackboard = blackboard
        self.xTarget, self.yTarget = util.world2map(locx, locy)
        #Warning: you must provide a location name
        #or the function will overwrite the old waypoints
        #For any previous location that you've generated.
        self.locKey = name or self.__class__.__name__
    
    #Execute and find a tree.
    def execute(self):
        #Run timestep to get readings from GPS
        util.robot.step(util.timestep)

        #Print and search for map file
        print("Generating route: ", self.locKey)
        file_exists = exists('cspace.npy')

        #Load map 
        if(not file_exists):
            print("Error: No map file detected")
            return False
        else:
            map = np.load('cspace.npy')
        
        #Get start and goal
        qstart = util.world2map(util.gps.getValues()[0],
        util.gps.getValues()[1])
        
        qgoal = (self.xTarget, self.yTarget)
        
        #Initialize graph
        G={qstart : []}
        
        #Vertex counter and max vertices
        k = 0
        K = 1000
        
        #Distance to search at
        Dq = 30 
        
        #Print       
        print("Generating route from ", qstart, "to ", qgoal)
        
        #Basically a do while loop:
        while(True):
        
            #Generate random location
            a = np.random.randint(200)
            b = np.random.randint(300)
            qrand = (a, b)
            
            #Randomly decide to go to the goal.
            if(np.random.rand() < 0.1):        
                qrand = qgoal  
                          
            #Get closest node to random location
            qnear = closestNode(G, qrand)
            
            #Get new node
            diff = np.subtract(qrand, qnear)
            qnew = qnear + (diff/mag(diff) * Dq)
            #Convert to int as we're using map coords
            qnew = qnew.astype(int)
            
            #Calculate distance from new node to goal
            gdist = np.sqrt((qnew[0]-qgoal[0])**2+(qnew[1]-qgoal[1])**2)
            
            #If we're close enough, just use the goal
            if(gdist < Dq):        
                qnew = qgoal
        
            #If the path is open, add to the dict and continue
            if(IsPathOpen(map, qnear, qnew)):
                G[tuple(qnew)] = []
                G[qnear].append(tuple(qnew))
                k += 1
                
            if(np.array_equal(qnew,qgoal) or k >= K):
                break
                
        #Print statement.
        print("Tree generated.")
        
        #Write the generated path to blackboard
        self.blackboard.write(self.locKey, generatePath(G, qstart, qgoal))
        
        #Return true
        return True

#Using map coordinates, finds if the path is open
def IsPathOpen(map,a,b):
    
    #Get coordinates of goal
    xGoal, yGoal = b
    
    #If the area is occupied, you can't go there. Return False.
    if(map[xGoal, yGoal]):
         return False
    
    #Generate a line from start to end
    (x,y) = line_nd(a, b ,integer=True)

    #Iterate over the line
    for i in range(len(x)):
        #If any point is occupied, return False
        if(map[x[i], y[i]]):
            return False
    
    #Return True
    return True

#Gets the size of a vector (in this case a tuple)    
def mag(x): 
    return np.sqrt(sum(i**2 for i in x))

#Gets closest node to x in G    
def closestNode(G, x):
    #closest distance is infinite, closest Node is none
    nearDist = float('Inf')
    nearNode = None
    
    #For every node in G:
    for key in G:
        #Calculate distance
        gdist=np.sqrt((key[0]-x[0])**2+(key[1]-x[1])**2)
        
        #If it's closer, update the nearest distance and node
        if gdist < nearDist:
            nearDist = gdist
            nearNode = key
    
    #Returns the nearest node        
    return nearNode

#Generates a path given map, start, and end.
#Uses AStar for an optimal route.
def generatePath(map,start,end):

    #Get queue
    queue = [(0, start)]
    heapify(queue)

    #Initialize distance and parent dicts.
    distances = defaultdict(lambda:float('Inf'))
    parent = {}

    #Set distance of start to 0, and add start to visited
    distances[start] = 0
    visited = {start}

    #Use the queue:    
    while(queue):
        #Pop and visit
        currentdist, v = heappop(queue)
        visited.add(v)
        
        #For all children:
        for u in map[v]:
        
            #If not visited:
            if u not in visited:
                #Find the updated cost using heuristic
                newcost = costH(u, v, end, distances)
                
                #If updated cost is closer, 
                if(newcost < distances[u]):
                    #Update distances, queue, and parent accordingly. 
                    distances[u] = newcost
                    heappush(queue, (newcost, u))
                    parent[u] = v
    
    #Generate path
    key = end
    path = []
                
    while key in parent:
        key = parent[key]
        path.insert(0, key)
    
    path.append(end)
    print("Path found: ", path)

    return convertPath(path)

#Converts a map-coordinate path to world coordinates
def convertPath(path):

    #Start with empty path
    worldPath = []

    #Iterate through path, and add converted version to 
    #the world coordinate path
    for xmap, ymap in path:
        xw, yw = util.map2world(xmap, ymap)
        worldPath.append((xw, yw))
    
    #Print and add a dummy coordinate to the end for driving
    print("Path in world coordinates: ", worldPath)
    worldPath.append((0, 0))
    
    #Return
    return worldPath

#Cost w/ heuristic
def costH(node, parent, goal, distances):

    #Base cost: Distance to parent from start
    dist1 = distances[parent]
    
    #Added cost: Distance to location from parent 
    dist2 = np.sqrt((node[0]-parent[0])**2+(node[1]-parent[1])**2)
    
    #Heuristic is euclidean distance
    h = np.sqrt((node[0]-goal[0])**2+(node[1]-goal[1])**2)

    #Generate cost and return.
    cost = dist1 + dist2 + h
    return cost
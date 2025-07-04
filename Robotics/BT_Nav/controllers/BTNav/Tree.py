#Imports all Robot items
import util

#Basic node definition
class Node():
    #Initialize with a list of children
    def __init__(self, children=None, name=None):
        self.children = children or []
        self.name = name or self.__class__.__name__

#Selector Definition
class Selector(Node):
    #Print execution
    def execute(self):
        print("Executing selector:", self.name)

        #Loop through children
        for child in self.children:
            #If one successfully executes, return True
            if child.execute():
                return True
        #Otherwise, return False
        return False

#Sequence Definition
class Sequence(Node):
    def execute(self):
        print("Executing sequence:", self.name)
        #Loop through children
        for child in self.children:
            #If one does not work, return False
            if not child.execute():
                return False
        #Otherwise, return True
        return True

#Parallel Node Definition
#A parallel node that succeeds once all children succeed
class Parallel(Node):

     #Execute
     def execute(self):
        print("Executing parallel:", self.name)
        results = [child.setup() for child in self.children]

        #While not ALL children are EXPLICITLY done, timestep and tick
        while(not all(result == True for result in results)):
            util.robot.step(util.timestep)
            #Update results
            results = [child.tick() for child in self.children]        
        
        #Terminate all children
        results = [child.terminate() for child in self.children]
        
        #Return
        return all(results)


#A parallel node that succeeds once one child succeeds
class ParallelOne(Node):
    
     #Execute
     def execute(self):
        #Print
        print("Executing parallelOne:", self.name)
        
        #Get initial results (this just runs setup() and is unimportant)
        results = [child.setup() for child in self.children]
        
        #While nothing is EXPLICITLY done, timestep, and tick
        while(not any(result == True for result in results)):
            util.robot.step(util.timestep)
            #Update results
            results = [child.tick() for child in self.children]        
        
        #Terminate all children
        results = [child.terminate() for child in self.children]
        #Return
        return any(results)
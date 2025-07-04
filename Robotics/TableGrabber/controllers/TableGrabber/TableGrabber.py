#The main behavior tree implementation.

#Selectors, sequences, and similar nodes are defined in Tree.py
import Tree

#Util contains the blackboard and other utility items
#Such as the robot declaration
import util

#Import driving, mapping, navigator classes to access their methods
import mapping
import driving
import navigator
import grasping

#Declare blackboard
blackboard = util.blackboard()

#Declare behavior tree
tree =  Tree.Sequence([

          #Stow arm and prepare to map. 
          grasping.stow("mapping prep"),
          
          #Selector node to test for map and map if not present
          Tree.Selector([mapping.TestForMap("Map Search"), 
                    Tree.ParallelOne([
                        driving.driveRobotWP("Circle Table", blackboard),
                        mapping.drawMap("Map", blackboard)])
                        ]),
          
          #Move to the general area to prepare to grab stuff
          navigator.generateWaypoints("prep", blackboard, 0, 0),
          driving.driveRobotWP("Prepare to grab", blackboard, "prep"),
           
          #Prepare arm for grabbing
          grasping.unstow("arm prep"),
          grasping.release("arm prep"),
          
          #Point to outside for scanning
          grasping.orient("outside", blackboard), 
           
          #Grab jam
          grasping.grab("jam", blackboard),
          
          #Point to table center to prepare to drop off
          grasping.orient("tableCenter", blackboard),

          #Drive to drop-off point and point to table center
          navigator.generateWaypoints("dropOff", blackboard, -0.5, 0.17),
          driving.driveRobotWP("Go to Drop Off", blackboard, "dropOff"),
          grasping.orient("tableCenter", blackboard),
          #Reuse unstow to reset the arm.
          #Unstow is used to lower the elbow, preventing a drop too far.
          grasping.unstow("prep item"), 
          #Release is used to release the jar
          grasping.release("prep item"), 
          #Lift elbow to clear obstacles
          grasping.lift("clear obstacles"),

          #Prepare to grab items again. Move to point
          navigator.generateWaypoints("prep", blackboard, 0, 0),
          driving.driveRobotWP("Prepare to grab", blackboard, "prep"),
           
          #Point to outside for scanning and reset arm
          grasping.orient("outside", blackboard),
          grasping.unstow("prep item"), 

           
          #Grab jam and point to center 
          grasping.grab("jam", blackboard),
          grasping.orient("tableCenter", blackboard),

          #Go to drop off and orient
          navigator.generateWaypoints("dropOff", blackboard, -0.5, 0.17),
          driving.driveRobotWP("Go to Drop Off", blackboard, "dropOff"),
          grasping.orient("tableCenter", blackboard),
          
          #Drop item and lift to clear obstacles
          grasping.unstow("prep item"), 
          grasping.release("prep item"), 
          grasping.lift("clear obstacles"),

          #Final preparation to get honey
          navigator.generateWaypoints("prep", blackboard, 0, 0),
          driving.driveRobotWP("Prepare to grab", blackboard, "prep"),
           
          #Point towards "outside" (ie. furnace hood)
          grasping.orient("outside", blackboard),
          
          #Unstow arm to prepare to grab 
          grasping.unstow("prep item"), 

          #Grab item and drop off
          grasping.grab("honey", blackboard),
          grasping.orient("tableCenter", blackboard),

          navigator.generateWaypoints("dropOff", blackboard, -0.5, 0.17),
          driving.driveRobotWP("Go to Drop Off", blackboard, "dropOff"),
          grasping.orient("tableCenter", blackboard),
          
          #Reuse unstow to reset the arm.
          grasping.unstow("prep item"), 
          grasping.release("drop"),
          grasping.lift("remove arm"),
          
          #Last part stores robot for fun.
          grasping.orient("back", blackboard),
          navigator.generateWaypoints("home", blackboard, -1.37, 0.28),
          driving.driveRobotWP("Go to Home", blackboard, "home"), 
          grasping.stow("store arm") 
        ])

#Execute the behavior tree
tree.execute()

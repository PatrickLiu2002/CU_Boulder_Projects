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

#Declare blackboard
blackboard = util.blackboard()

#Declare behavior tree
tree =  Tree.Sequence([Tree.Selector([mapping.TestForMap("Map Search"), 
                    Tree.ParallelOne([
                        driving.driveRobotWP("Circle Table", blackboard),
                        mapping.drawMap("Map", blackboard)])
                        ]),
           navigator.generateWaypoints("lowerLeft", blackboard, -1.56, 0.43),
           driving.driveRobotWP("Go to Lower Left", blackboard, "lowerLeft"),
           navigator.generateWaypoints("sink", blackboard, 0.59, 0.43),
           driving.driveRobotWP("Go to Sink", blackboard, "sink"),
        ])

#Execute the behavior tree
tree.execute()

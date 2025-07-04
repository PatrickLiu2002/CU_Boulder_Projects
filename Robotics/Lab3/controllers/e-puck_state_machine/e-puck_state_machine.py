#Import robot from controller
from controller import Robot

#Initialize robot instances
robot = Robot()

#Set up the timestep stuff
timestep = int(robot.getBasicTimeStep())

#Get left/right wheels
motor_left = robot.getDevice("left wheel motor")
motor_right = robot.getDevice("right wheel motor")

#Set left/right wheels to turn
motor_left.setPosition(float('Inf'))
motor_right.setPosition(float('Inf'))

#Set the max speed to 2 rad/s
MAX_SPEED = 6.28

#Initialize sensors. 

#Distance sensors activated.
ps = []
for i in range(8):
    ps.append(robot.getDevice("ps" + str(i)))
    ps[-1].enable(timestep)

#Print message
print("All sensors online.")

#Initialize state to first state
state = "CHARGE"

#This is the intensity for 0.05m
#I was able to measure this.
#Being above this intensity means the distance is closer
#Than 0.05m (roughly)
desiredDist = 1000

#Main control loop
while robot.step(timestep) != -1:
    values = []
    for distanceSensor in ps:
        values.append(distanceSensor.getValue())
        
    #State 1: Drive forward until sensor reads <0.05
    #This state is complete.
    if(state == "CHARGE"):
        #drive
        #print(values)
        motor_left.setVelocity(MAX_SPEED)
        motor_right.setVelocity(MAX_SPEED)
        #If ps is less than or equal to desired reading
        #Go to next state
        if(values[0] >= desiredDist and values[7] >= desiredDist):
            motor_left.setVelocity(0)
            motor_right.setVelocity(0)
            state = "SPIN"
            #The "continue" statement is needed to update
            #the sensor values.
            continue
     #End State 1 Handler
     
    #Begin State 2 Handler
    if(state == "SPIN"):
        #Set to spin clockwise
        motor_left.setVelocity(MAX_SPEED/10)
        motor_right.setVelocity(-MAX_SPEED/10)
        
        #By timing it out, we can find the time needed.
        robot.step(7050)
        state = "MOVE"
        continue
         #robot.step(timestep)
        

         
     #go to MOVE state, but modified
     #if(state == "MOVE"):
         #Spin
    #This is just a rehash of CHARGE, but it will
    #Go to TURN instead of SPIN after reaching <0.05m
    if(state == "MOVE"):
        motor_left.setVelocity(MAX_SPEED)
        motor_right.setVelocity(MAX_SPEED)
        #If ps is less than or equal to desired reading
        if(values[7] >= desiredDist and values[0] >= desiredDist):
            motor_left.setVelocity(0)
            motor_right.setVelocity(0)
            #Delay to allow the robot to stop fully.
            robot.step(100)
            state = "TURN"
            continue
         
    #State: Spin again
     
    if (state == "TURN"):
        motor_left.setVelocity(MAX_SPEED/10)
        motor_right.setVelocity(-MAX_SPEED/10)
        if(values[5] >= desiredDist):
            state = "FOLLOW"
            continue

     #go to FOLLOW state
    if (state == "FOLLOW"):
    #Drives along the wall until needed. 
        motor_left.setVelocity(MAX_SPEED)
        motor_right.setVelocity(MAX_SPEED)
        if(values[5] < desiredDist - 500):
            state = "STOP"
            continue


    if (state == "STOP"):
        motor_left.setVelocity(0)
        motor_right.setVelocity(0)
        continue


    pass
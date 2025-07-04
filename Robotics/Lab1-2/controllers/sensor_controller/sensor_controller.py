from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

motor_left = robot.getDevice("left wheel motor")
motor_right = robot.getDevice("right wheel motor")

motor_left.setPosition(float('Inf'))
motor_right.setPosition(float('Inf'))

ls = []
for i in range(8):
    ls.append(robot.getDevice("ls" + str(i)))
    ls[-1].enable(timestep)

print("All sensors online.")

while robot.step(timestep) != -1:
    values = []
    for lightsensor in ls:
        values.append(lightsensor.getValue())
    
    print(values)
    
    motor_left.setVelocity(values[7]/1000)
    motor_right.setVelocity(values[0]/1000)

    pass
"""sumo_fight controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import TouchSensor
from json.encoder import INFINITY

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#Getting the robot wheels
wheels = []
wheelsNames = ['left wheel motor', 'right wheel motor']
for i in range(len(wheelsNames)):
    wheels.append(robot.getDevice(wheelsNames[i]))

leftSpeed = 0.0
rightSpeed = 0.0
wheels[0].setPosition(INFINITY)
wheels[1].setPosition(INFINITY)

while robot.step(timestep) != -1:
    leftSpeed = 5.0
    rightSpeed = 5.0
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)

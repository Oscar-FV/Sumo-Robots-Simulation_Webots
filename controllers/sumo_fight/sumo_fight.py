"""sumo_fight controller."""
"""Oscar Flores Vázquez - 280210"""
"""Robots Fighiting Simulation """

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from gettext import find
from controller import Robot
from controller import TouchSensor
from controller import DistanceSensor
from json.encoder import INFINITY
from fightingFunctions import *
from basicFunctions import *

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#Getting groundSensor
#The ground sensor is a infrared DistanceSensor pointing downwards
irSensor = robot.getDevice('gs0')
irSensor.enable(timestep)

# #Getting distanceSensor to detect if the robot is getting close to the end of the arena
distanceSensors_Names = ['gs1', 'gs2'] 
distanceSensors = []
for i in range(len(distanceSensors_Names)):
    distanceSensors.append(robot.getDevice(distanceSensors_Names[i]))
    distanceSensors[i].enable(timestep)

#Getting the robot wheels
wheels = []
wheelsNames = ['left wheel motor', 'right wheel motor']
for i in range(len(wheelsNames)):
    wheels.append(robot.getDevice(wheelsNames[i]))

leftSpeed = 0.0
rightSpeed = 0.0
wheels[0].setPosition(INFINITY)
wheels[1].setPosition(INFINITY)

lineFlag = False

while robot.step(timestep) != -1:

    leftSpeed = 3
    rightSpeed = 3  
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)

    wheels[0].setPosition(INFINITY)
    wheels[1].setPosition(INFINITY)

    
    # ground sensor to look for the whitestripe, stops for 5 seconds the the fight starts
    if lineFlag == False:
        lineFlag = whiteLineIsFound(robot, wheels, irSensor)

    #checking the distance sensors that are pointing downwards to detect if the arena is ending
    cliffDetection(robot, wheels, distanceSensors)


    

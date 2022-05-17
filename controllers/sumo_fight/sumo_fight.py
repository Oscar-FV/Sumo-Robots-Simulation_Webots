"""sumo_fight controller."""
"""Oscar Flores Vázquez - 280210"""
"""Robots Fighiting Simulation """

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import TouchSensor
from controller import DistanceSensor
from json.encoder import INFINITY
from fightingFunctions import *

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#Getting groundSensor
#The ground sensor is a infrared DistanceSensor pointing downwards
groundSensor = robot.getDevice('gs0')
groundSensor.enable(timestep)

#Getting touchSensor, for this project i used a touchSensor of tipe "force"
touchSensor = robot.getDevice('touch sensor')
touchSensor.enable(timestep)

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
    # if(groundSensor.getValue() < 5.6):
    #     leftSpeed = 5.0
    #     rightSpeed = 5.0
    # else:
    #     leftSpeed = 0
    #     rightSpeed = 0

    # wheels[0].setVelocity(leftSpeed)
    # wheels[1].setVelocity(rightSpeed)


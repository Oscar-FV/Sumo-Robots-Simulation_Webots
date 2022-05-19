"""sumo_fight controller."""
"""Oscar Flores Vázquez - 280210"""
"""Robots Fighiting Simulation """

# create the Robot instance.
from controller import Robot
from controller import TouchSensor
from controller import DistanceSensor
from controller import Camera
from controller import CameraRecognitionObject
from json.encoder import INFINITY
import random
from fightingFunctions import *
from basicFunctions import *

robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Getting groundSensor
# The ground sensor is a infrared DistanceSensor pointing downwards
irSensor = robot.getDevice('gs0')
irSensor.enable(timestep)

# Getting distanceSensor to detect if the robot is getting close to the end of the arena
distanceSensors_Names = ['gs1', 'gs2']
distanceSensors = []
for i in range(len(distanceSensors_Names)):
    distanceSensors.append(robot.getDevice(distanceSensors_Names[i]))
    distanceSensors[i].enable(timestep)

#Getting the touchSensor this sensor is the type "bumper", if the robot is touching something it returns a 1
touchSensor = robot.getDevice('touch sensor')
touchSensor.enable(timestep)

# Getting the robot camera
camera = robot.getDevice('cam')
camera.enable(timestep)
camera.recognitionEnable(timestep)

# getting the center of the camera image
cameraMiddle = []
cameraMiddle.append(camera.getWidth()/2)
cameraMiddle.append(camera.getHeight()/2)

# Getting the robot wheels
wheels = []
wheelsNames = ['left wheel motor', 'right wheel motor']
for i in range(len(wheelsNames)):
    wheels.append(robot.getDevice(wheelsNames[i]))

wheels[0].setPosition(INFINITY)
wheels[1].setPosition(INFINITY)

lineFlag = False
enemyBump = False

#array with diferent velocitys, a number will get generated randomly in the range of the array lenght
#velocity 2-3 is a softpunch
#velocity 4-5 is a mediumpunch
#velocity 6 is a hardpunch
speedsArray = [2,2,2,3,3,3,3,4,4,5,5,6]
speedsArray2 = [2,3,3,4,4,4,5,5,5,5,6,6]
speedsArray3 = [2,2,2,3,3,3,3,6,6,6,6,6]
speed = 4.0

while robot.step(timestep) != -1:

    wheels[0].setVelocity(speed)
    wheels[1].setVelocity(speed)

    # ground sensor to look for the whitestripe, stops for 5 seconds the the fight starts
    if lineFlag == False:
        lineFlag = whiteLineIsFound(robot, wheels, irSensor)
    else:

        if enemyBump == True:
            if speed == 2 or speed == 3:
                goBackwards(robot, wheels, 1)
                speed = random.choice(speedsArray)
            else:
                if speed == 4 or speed == 5:
                    goBackwards(robot, wheels, 0.2)
                    speed = random.choice(speedsArray2)
                else:
                    speed = random.choice(speedsArray3)
                    enemyBump == True
        
        if touchSensor.getValue() == 1:
            enemyBump = True
        else:
            enemyBump = False

        #checking the distance sensors that are pointing downwards to detect if the arena is ending
        cliffDetection(robot, wheels, distanceSensors, 2)

        if camera.getRecognitionNumberOfObjects() > 0:
            cameraRecognition = camera.getRecognitionObjects()
            objectPosition = cameraRecognition[0].get_position_on_image()

            if objectPosition[0] > cameraMiddle[0]:
                wheels[0].setVelocity(speed)
                wheels[1].setVelocity(-speed)
                if objectPosition[0] <= cameraMiddle[0] + 10:
                    wheels[0].setVelocity(speed)
                    wheels[1].setVelocity(speed)
            else:
                wheels[0].setVelocity(-speed)
                wheels[1].setVelocity(speed)
                if objectPosition[0] >= cameraMiddle[0] - 10:
                    wheels[0].setVelocity(speed)
                    wheels[1].setVelocity(speed)
        else:
            wheels[0].setVelocity(-speed)
            wheels[1].setVelocity(speed)
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

leftSpeed = 0.0
rightSpeed = 0.0
wheels[0].setPosition(INFINITY)
wheels[1].setPosition(INFINITY)

lineFlag = False
enemyBump = False

while robot.step(timestep) != -1:

    leftSpeed = 6
    rightSpeed = 6
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)

    wheels[0].setPosition(INFINITY)
    wheels[1].setPosition(INFINITY)

    # # ground sensor to look for the whitestripe, stops for 5 seconds the the fight starts
    # if lineFlag == False:
    #     lineFlag = whiteLineIsFound(robot, wheels, irSensor)
    # else:

    #     #checking the distance sensors that are pointing downwards to detect if the arena is ending
    #     cliffDetection(robot, wheels, distanceSensors, 2)

    #goTowardsEnemy(wheels, camera, cameraMiddle[0])

    if camera.getRecognitionNumberOfObjects() > 0:
        cameraRecognition = camera.getRecognitionObjects()
        objectPosition = cameraRecognition[0].get_position_on_image()

        if objectPosition[0] > cameraMiddle[0]:
            wheels[0].setVelocity(1)
            wheels[1].setVelocity(-6)
            if objectPosition[0] <= cameraMiddle[0] + 5:
                wheels[0].setVelocity(6)
                wheels[1].setVelocity(6)
        else:
            wheels[0].setVelocity(-1)
            wheels[1].setVelocity(6)
            if objectPosition[0] >= cameraMiddle[0] - 5:
                wheels[0].setVelocity(6)
                wheels[1].setVelocity(6)
    else:
        wheels[0].setVelocity(-6)
        wheels[1].setVelocity(1)
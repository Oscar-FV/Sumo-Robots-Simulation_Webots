from controller import Robot
from controller import TouchSensor
from controller import DistanceSensor

#----------Functions that allow us to make certain instructions for a few secods--------#
def getTimeStep(robot):
    timeStep = -1
    if timeStep == -1:
        timeStep = robot.getBasicTimeStep()
    return int(timeStep)

def Step(robot):
    if robot.step(getTimeStep(robot)) == -1:
        robot.__del__()
        exit()

def passiveWait(robot, sec):
    startTime = robot.getTime()
    while (startTime + sec > robot.getTime()):
        Step(robot)
#---------------------------------------------------------------------------------------#

#---------------------------Basic movement Functions------------------------------------#
def goBackwards(robot, wheels):
    leftSpeed = -3.0
    rightSpeed = -3.0
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    passiveWait(robot, 2)

def turnLeft(robot, wheels):
        leftSpeed = -3.0
        rightSpeed = 3.0
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)
        passiveWait(robot, 3)
#---------------------------------------------------------------------------------------#

#--------------------------------Detection Functions------------------------------------#

#Detects a white line, stops for 5 seconds then keeps going forward
#The sensor that is used for this detection is a infrared sensor, when the sensor detects the colos white it returns a value around 96
def whiteLineIsFound(robot, wheels, groundSensor):
    
    if(groundSensor.getValue() < 97):
        wheels[0].setVelocity(0)
        wheels[1].setVelocity(0)
        passiveWait(robot,5)
        return True
    else:
        return False

    #---------------------------------------------------------------------------------------#

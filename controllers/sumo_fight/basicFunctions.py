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

def findWhiteLine(groundSensor):
    
    if(groundSensor.getValue() < 97):
        return True
    else:
        return False
    
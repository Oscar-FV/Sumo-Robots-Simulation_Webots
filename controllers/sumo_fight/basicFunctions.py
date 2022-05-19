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
def goBackwards(robot, wheels, sec):
    wheels[0].setVelocity(-3.0)
    wheels[1].setVelocity(-3.0)
    passiveWait(robot, sec)

def turnLeft(robot, wheels):
        wheels[0].setVelocity(-3.0)
        wheels[1].setVelocity(3)
        passiveWait(robot, 1)

def turnRight(robot, wheels):
        wheels[0].setVelocity(3.0)
        wheels[1].setVelocity(-3.0)
        passiveWait(robot, 1)
#---------------------------------------------------------------------------------------#

#--------------------------------Detection Functions------------------------------------#

#Detects a white line, stops for 5 seconds then keeps going forward
#The sensor that is used for this detection is a infrared sensor, when the sensor detects the colos white it returns a value around 96
def whiteLineIsFound(robot, wheels, irSensor):
    
    if(irSensor.getValue() > 500):
        wheels[0].setVelocity(0)
        wheels[1].setVelocity(0)
        passiveWait(robot,5)
        return True
    else:
        return False

#Detects if the area of the arena is ending, if one of the two sensors detects the end of the arena, the robot goes backwards, and then
#depending of the sensor that detects the cliff (left or right) turn in the opposite direction
def cliffDetection(robot, wheels, distanceSensors, sec):
    for i in range(len(distanceSensors)):
        if distanceSensors[i].getValue() > 200:
            goBackwards(robot, wheels, sec)
            if(i == 0):
                turnLeft(robot, wheels)
            else:
                turnRight(robot, wheels)

    #---------------------------------------------------------------------------------------#

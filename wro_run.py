from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, multitask, run_task
import umath
hub = PrimeHub()

left_motor = Motor(Port.E, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.F)
color_sensor_front = ColorSensor(Port.A)
color_sensor = ColorSensor(Port.B)
front_arm = Motor(Port.D)
back_arm = Motor(Port.C, Direction.COUNTERCLOCKWISE)
drive_base = DriveBase(left_motor, right_motor, wheel_diameter=55, axle_track=196)
drive_base.use_gyro(True)
WHEEL_SIZE = 17.5
DEGREES_PER_CM = 360/WHEEL_SIZE
yellowAtFront = -1
greenAtFront = -1
print("START")
print("--------")
def SetGyro(truefalse):
    drive_base.use_gyro(truefalse)

def SetSpeed(speed):
    drive_base.settings(straight_speed=speed)


def SetAccel(accelStraight, accelTurn):
    drive_base.settings(straight_acceleration=accelStraight, turn_acceleration=accelTurn)

def MoveForward(distance):
    drive_base.straight(distance)

def MoveForwardWithAccel(distance, accelStraight, accelTurn, dir):
    drive_base.settings(straight_acceleration=accelStraight, turn_acceleration=accelTurn)
    drive_base.straight(distance*dir)

def AccelDefaultReset():
    SetAccel(733,763)

def MoveBackward(distance):
    drive_base.straight(-1* distance)

def TurnRight(degrees):
    drive_base.turn(degrees)

def TurnLeft(degrees):
    drive_base.turn(-1* degrees)

def StopAtWhite(speed):
    drive_base.drive(speed, 0)
    while(not (color_sensor.hsv().h >= 200 and color_sensor.hsv().h <= 300)):
        pass
    drive_base.brake()

def PauseMission():
    print("start pause mission")
    SetGyro(False)
    while Button.LEFT not in hub.buttons.pressed():
        wait(10)
    print("pause mission end")
    SetGyro(True)

def MoveForwardWithSuddenStop(distance):
    drive_base.use_gyro(True)
    drive_base.drive(800, 0)
    while drive_base.distance()< distance : 
        print(drive_base.distance())
        wait(20)
    drive_base.straight(0)
def lineFollowWithDistance(dist, speed):
    drive_base.drive(speed, 0)
    print(drive_base.distance())
    while drive_base.distance() < dist:
        print(color_sensor.reflection())
        drive_base.drive(speed, ((-3/5)*color_sensor.reflection()+30))
    drive_base.stop()
def MoveArmWithStallTimeDetection(arm, speed, angle, time):
    arm.reset_angle(0)
    arm.run(speed)
    watch = StopWatch()
    while abs(arm.angle()) < abs(angle):
        if watch.time() > time:
            break
    arm.hold()
def FindHSVRange():
    highestH = 0
    lowestH = 1000
    highestS = 0
    lowestS = 1000
    while(True):
        detectedHsv = color_sensor_front.hsv()
        detectedColor = color_sensor_front.color()
        highestH = max(highestH, detectedHsv.h)
        lowestH = min(lowestH, detectedHsv.h)
        highestS = max(highestS, detectedHsv.s)
        lowestS = min(lowestS, detectedHsv.s)
        print(detectedHsv.h)
        wait(10)
def grabRight():
    front_arm.stop()
    MoveArmWithStallTimeDetection(back_arm, 350, 180, 500)
    MoveForward(80)
    front_arm.run_angle(500, -70)
    drive_base.turn(-20)
    MoveForward(70)
    front_arm.run_angle(500, 70)    
    MoveArmWithStallTimeDetection(back_arm, -350, 180, 500)
    MoveBackward(70)
    drive_base.turn(30)
    drive_base.turn(-10)
    MoveBackward(80)
    
def grabLeft():
    MoveArmWithStallTimeDetection(back_arm, 200, 140, 500)
    MoveForward(75)
    drive_base.turn(20)
    MoveForward(65)
    MoveArmWithStallTimeDetection(back_arm, -200, 140, 500)
    MoveBackward(65)
    drive_base.turn(-30)
    drive_base.turn(10)
    MoveBackward(75)

def isRed(hVal):
    return hVal >= 340 and hVal <= 360
    
def isWhite(hVal):
    return hVal >= 180 and hVal <= 270

def isYellow(hVal):
    return hVal >= 45 and hVal <= 55

def isGreen(hVal):
    return hVal >= 150 and hVal <= 170

async def moveArmUp(arm, speed, angle):
    await arm.run_angle(speed, angle)
   
async def moveArmDown(arm, speed, angle):
    await arm.run_angle(speed, -1 * angle)

async def MoveForward_As(distance):
    await drive_base.straight(distance)

async def DriveForwardAndMoveArm(distance, arm, armSpeed, angle):
    await multitask(MoveForward_As(distance), moveArmUp(arm, armSpeed, angle))


#making its way to the balls
run_task(DriveForwardAndMoveArm(-250, front_arm, 500, -105))
drive_base.turn(-90)
MoveForward(-300)
drive_base.turn(-90)

#This code below collects the balls with the front arm
MoveForwardWithAccel(175, 2000, 500, -1)
MoveForwardWithAccel(100, 1000, 500, 1)
MoveForwardWithAccel(110, 2000, 500, -1)
MoveForwardWithAccel(240, 1000, 500, 1)

#getting to mars rover and completing it
back_arm.run_angle(500, -75)
MoveArmWithStallTimeDetection(front_arm, 100, 30, 500)
MoveForward(-130)

#getting to the container
drive_base.turn(-90)
run_task(DriveForwardAndMoveArm(410, back_arm, 150, 75))
drive_base.turn(-90)
run_task(DriveForwardAndMoveArm(-85, back_arm, 150, -100))

#This code below opens the door with the back arm
MoveForward(50)
MoveBackward(10)
MoveArmWithStallTimeDetection(back_arm, 100, 5, 500)
run_task(DriveForwardAndMoveArm(110, back_arm, 70, 30))
MoveForward(20)

#front_arm.run_angle(350, 130)
MoveArmWithStallTimeDetection(front_arm, 300, 70, 500)
wait(1000)
'''
#This code aligns against the wall and travels to the colored blocks
MoveArmWithStallTimeDetection(front_arm, -100, 100, 500)
SetSpeed(600)
MoveBackward(300)
MoveArmWithStallTimeDetection(front_arm, 100, 100, 500)
back_arm.run_angle(100, 50)
SetSpeed(200)
drive_base.turn(-90)
SetGyro(False)
MoveBackward(300)
SetGyro(True)
MoveForward(80)
drive_base.turn(90)
StopAtWhite(-150)
MoveForward(30)
drive_base.turn(-90)

SetGyro(False)
MoveBackward(150)
SetGyro(True)
MoveArmWithStallTimeDetection(back_arm, -350, 180, 500)
'''
'''
#This code picks up the colored blocks
for i in range(6):
    hVal = color_sensor_front.hsv().h
    if(isRed(hVal)):
        print("red")
        grabLeft()
        if(yellowAtFront == -1):
            yellowAtFront = 1
    elif(isWhite(hVal)):
        print("white")
        grabRight()
        if(greenAtFront == -1):
            greenAtFront = 1
    elif(isGreen(hVal)):
        print("green")
        grabRight()
        if(greenAtFront == -1):
            greenAtFront = 0
    elif(isYellow(hVal)):
        print("yellow")
        grabLeft()
        if(yellowAtFront == -1):
            yellowAtFront = 0
    else:
        print("none")
    MoveForward(97)

#this code pushes the drone into the home area
drive_base.turn(-40)
MoveForward(450)
drive_base.turn(-50)
SetSpeed(500)
MoveBackward(1030)
SetSpeed(200)

#navigation to dropoff of blocks
MoveForward(200)
drive_base.turn(-100)
SetGyro(False)
MoveBackward(300)
SetGyro(True)
if(yellowAtFront):
    MoveForward(170)
else:
    MoveForward(150)
drive_base.turn(90)
SetSpeed(600)
MoveForward(600)

#drop off yellow and red blocks
if(yellowAtFront == 1):
    SetSpeed(200)
    StopAtWhite(200)
    MoveForward(625)
    front_arm.stop()
    MoveArmWithStallTimeDetection(back_arm, 200, 90, 500)
    MoveBackward(50)
    MoveArmWithStallTimeDetection(back_arm, -200, 90, 500)
    MoveBackward(100)
    drive_base.turn(28)
    MoveForward(50)
    MoveArmWithStallTimeDetection(back_arm, 200, 90, 500)
    MoveBackward(80)

    MoveArmWithStallTimeDetection(back_arm, -200, 90, 500) 
    MoveForward(140)
    MoveBackward(140)
    drive_base.turn(-28)
    MoveBackward(200)
else:
    SetSpeed(200)
    StopAtWhite(200)
    MoveForward(550)
    front_arm.stop()
    drive_base.turn(32)
    MoveArmWithStallTimeDetection(back_arm, 200, 90, 500)
    MoveBackward(40)
    MoveArmWithStallTimeDetection(back_arm, -200, 90, 500)
    MoveForward(70)
    MoveBackward(80)
    drive_base.turn(-32)
    MoveArmWithStallTimeDetection(back_arm, 200, 90, 500)
    MoveBackward(100)
    MoveArmWithStallTimeDetection(back_arm, -200, 90, 500)
    MoveForward(160)
    MoveBackward(350)

#navigate to white and green block drop off area
StopAtWhite(-200)
MoveBackward(150)
drive_base.turn(-90)
SetGyro(False)
MoveBackward(350)
SetGyro(True)
drive_base.stop()
SetSpeed(600)
MoveForward(300)
StopAtWhite(300)
SetSpeed(600)
run_task(DriveForwardAndMoveArm(430, back_arm, 100, 90))
drive_base.brake()
SetSpeed(200)
drive_base.turn(90)
SetSpeed(200)

#drop off white and green blocks
if(greenAtFront == 1):
    MoveForward(540)
    MoveArmWithStallTimeDetection(front_arm, -200, 70, 500)
    MoveBackward(50)
    MoveArmWithStallTimeDetection(front_arm, 200, 70, 500)
    MoveBackward(210)
    drive_base.turn(-21)
    MoveForward(150)
    MoveArmWithStallTimeDetection(front_arm, -200, 70, 500)
    MoveBackward(80)
    MoveArmWithStallTimeDetection(front_arm, 200, 70, 500)
    MoveForward(170)
else:
    MoveForward(340)
    drive_base.turn(-24)
    MoveForward(100)
    MoveArmWithStallTimeDetection(front_arm, -200, 70, 500)
    MoveBackward(50)
    MoveArmWithStallTimeDetection(front_arm, 200, 70, 500)
    MoveForward(120)
    MoveBackward(100)
    drive_base.turn(24)
    MoveForward(165)
'''


#discarded (very last part)

'''
MoveForward(20)
MoveBackward(90)
drive_base.turn(30)
MoveForward(40)
MoveArmWithStallTimeDetection(back_arm, 200, 70, 500)
MoveBackward(80)
MoveArmWithStallTimeDetection(back_arm, -200, 50, 500)
MoveForward(120)
drive_base.turn(5)
SetSpeed(600)
MoveBackward(100)
drive_base.turn(-35)
run_task(DriveForwardAndMoveArm(-300, front_arm, 100, 90))

front_arm.run_angle(100, -90)
drive_base.turn(90)
SetGyro(False)
MoveBackward(400)
SetGyro(True)
MoveForward(530)
drive_base.brake()
drive_base.turn(-90)
SetSpeed(200)
SetSpeed(200)
MoveForward(700)
'''
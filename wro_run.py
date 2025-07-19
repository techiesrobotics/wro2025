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
print("START: ")
print("--------")
# Print the current battery voltage in millivolts
print("Battery voltage:", hub.battery.voltage())
'''
Voltage (mV)
Status
Notes
~8200–8600
🔋 Full
Fully charged
~7600–8000
🟡 Normal
Medium charge
~7000–7500
🔻 Low
Consider recharging soon
< 7000
⚠️ Critical
Very low; may experience performance drops
'''
def SetGyro(truefalse):
    drive_base.use_gyro(truefalse)

def SetSpeed(speed):
    drive_base.settings(straight_speed=speed)

def SetTurnSpeed(speed):
    drive_base.settings(turn_rate=speed)

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
    drive_base.reset()
    while abs(drive_base.distance()) < distance : 
        print(drive_base.distance())
        wait(5)
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
        print(detectedHsv.h, " | ", detectedHsv.s, " | ", detectedHsv.v)
        wait(10)
def grabRight(i):
    front_arm.stop()
    MoveArmWithStallTimeDetection(back_arm, 200, 35, 500)
    MoveForward(80)
    SetTurnSpeed(80)
    front_arm.run_angle(500, -70)
    drive_base.turn(-20)
    if(i >= 4):
        MoveForward(50)
    else:
        MoveForward(50)
    front_arm.run_angle(200, 70)    
    MoveArmWithStallTimeDetection(back_arm, -200, 70, 500)
    if(i >= 4):
        MoveBackward(50)
    else:
        MoveBackward(50)
    drive_base.turn(30)
    drive_base.turn(-10)
    MoveBackward(80)
    SetTurnSpeed(100)
    
def grabLeft(i):
    MoveArmWithStallTimeDetection(back_arm, 200, 140, 500)
    MoveForward(77)
    drive_base.turn(21)
    if(i >= 4):
        MoveForward(60)
    else:
        MoveForward(60)
    MoveArmWithStallTimeDetection(back_arm, -200, 170, 500)
    if(i >= 4):
        MoveBackward(60)
    else:
        MoveBackward(60)
    drive_base.turn(-31)
    drive_base.turn(10)
    MoveBackward(77)

def isRed(hVal):
    return hVal >= 340 and hVal <= 360
    
def isWhite(hVal, sVal):
    return (hVal >= 180 and hVal <= 270) or (sVal >= 7 and sVal <= 11)

def isYellow(hVal):
    return hVal >= 45 and hVal <= 55

def isGreen(hVal):
    return hVal >= 140 and hVal <= 170

async def moveArmUp(arm, speed, angle):
    await arm.run_angle(speed, angle)
   
async def moveArmDown(arm, speed, angle):
    await arm.run_angle(speed, -1 * angle)

async def MoveForward_As(distance):
    await drive_base.straight(distance)

async def DriveForwardAndMoveArm(distance, arm, armSpeed, angle):
    await multitask(MoveForward_As(distance), moveArmUp(arm, armSpeed, angle))
'''
def test(hVal, sVal):
    print(hVal, sVal)
    hVal = color_sensor_front.hsv().h
    sVal = color_sensor_front.hsv().s
    if(isRed(hVal)):
        print("red")
    elif(isWhite(hVal, sVal)):
        print("white")
    elif(isGreen(hVal)):
        print("green")
    elif(isYellow(hVal)):
        print("yellow")
    else:
        print("none")
hVal = color_sensor_front.hsv().h
sVal = color_sensor_front.hsv().s
test(hVal, sVal)
'''


#making its way to the balls
run_task(DriveForwardAndMoveArm(-250, front_arm, 500, -105))
drive_base.turn(-90)
MoveForward(-298)
drive_base.turn(-90)

#This code below collects the balls with the front arm
MoveForwardWithAccel(185, 2000, 500, -1)
MoveArmWithStallTimeDetection(front_arm, -100, 10, 200)
MoveForwardWithAccel(40, 1000, 500, 1)
MoveArmWithStallTimeDetection(front_arm, -100, 10, 200)
MoveForwardWithAccel(110, 1000, 500, -1)
MoveArmWithStallTimeDetection(front_arm, -100, 10, 200)
MoveForwardWithAccel(100, 1000, 500, 1)
MoveArmWithStallTimeDetection(front_arm, 100, 30, 500)
MoveForward(160)
#getting to mars rover and completing it
back_arm.run_angle(500, -75)
MoveForward(-120)
#getting to the container
drive_base.turn(-90)
run_task(DriveForwardAndMoveArm(425, back_arm, 150, 75))
drive_base.turn(-90)
run_task(DriveForwardAndMoveArm(-85, back_arm, 100, -105))

#This code below opens the door with the back arm
MoveForward(55)
MoveBackward(7)
MoveArmWithStallTimeDetection(back_arm, 100, 10, 500)
run_task(DriveForwardAndMoveArm(110, back_arm, 55, 30))
MoveForward(20)
MoveBackward(20)

#front_arm.run_angle(350, 130)
MoveArmWithStallTimeDetection(front_arm, 300, 70, 500)
wait(1000)

#This code aligns against the wall and travels to the colored blocks
MoveArmWithStallTimeDetection(front_arm, -100, 100, 500)
SetSpeed(600)
MoveBackward(300)
MoveArmWithStallTimeDetection(front_arm, 100, 130, 500)
back_arm.run_angle(100, 50)
SetSpeed(300)
drive_base.turn(-90)
SetGyro(False)
MoveBackward(300)
SetGyro(True)
SetSpeed(200)
MoveForward(70)
drive_base.turn(90)
StopAtWhite(-150)
MoveForward(27)
drive_base.turn(-90)

SetGyro(False)
MoveBackward(150)
SetGyro(True)
MoveArmWithStallTimeDetection(back_arm, -350, 180, 500)
MoveArmWithStallTimeDetection(front_arm, 200, 50, 500)
whiteCnt = 0
#PauseMission()
#This code picks up the colored blocks
MoveForward(5)
for i in range(6):
    hVal = color_sensor_front.hsv().h
    sVal = color_sensor_front.hsv().s
    if(isRed(hVal)):
        print("red")
        grabLeft(i)
        if(yellowAtFront == -1):
            yellowAtFront = 1
    elif(isWhite(hVal, sVal)):
        print("white")
        grabRight(i)
        whiteCnt = whiteCnt + 1
        if(greenAtFront == -1):
            greenAtFront = 1
    elif(isGreen(hVal)):
        print("green")
        grabRight(i)
        if(greenAtFront == -1):
            greenAtFront = 0
    elif(isYellow(hVal)):
        print("yellow")
        grabLeft(i)
        if(yellowAtFront == -1):
            yellowAtFront = 0
    else:
        print("none")
    if(whiteCnt > 1):
        greenAtFront = 0
    if(i <= 2):
        MoveForward(95)
    elif(i >= 3 and i < 4):
        MoveForward(91)
    elif(i == 4):
        MoveForward(103)
    else:
        MoveForward(91)
    if(i == 1 or i == 3):
        drive_base.turn(1)
    print(hVal, sVal)

#this code pushes the drone into the home area
SetTurnSpeed(300)
SetSpeed(450)
drive_base.turn(-35)
MoveForward(430)
drive_base.turn(-55)
MoveBackward(1025)

SetTurnSpeed(100)
#navigation to dropoff of blocks
SetSpeed(300)
MoveForward(200)
drive_base.turn(-90)
SetGyro(False)
MoveBackward(300)
SetGyro(True)
SetSpeed(200)
SetTurnSpeed(100)
if(yellowAtFront == 1):
    MoveForward(175)
else:
    MoveForward(160)
drive_base.turn(90)
SetSpeed(600)
MoveForward(600)

#drop off yellow and red blocks
if(yellowAtFront == 1):
    SetSpeed(200)
    StopAtWhite(200)
    MoveForward(635)
    front_arm.stop()
    MoveArmWithStallTimeDetection(back_arm, 200, 90, 500)
    MoveBackward(50)
    MoveArmWithStallTimeDetection(back_arm, -120, 90, 500)
    MoveBackward(30)
    MoveArmWithStallTimeDetection(back_arm, -30, 90, 500)
    MoveBackward(40)
    SetTurnSpeed(70)
    drive_base.turn(35)
    SetTurnSpeed(100)
    MoveForward(50)
    MoveArmWithStallTimeDetection(back_arm, 150, 90, 500)
    MoveBackward(110)

    MoveArmWithStallTimeDetection(back_arm, -200, 90, 500) 
    MoveForward(120)
    MoveBackward(90)
    drive_base.turn(-35)
    MoveBackward(200)
else:
    SetSpeed(200)
    StopAtWhite(200)
    MoveForward(540)
    front_arm.stop()
    drive_base.turn(35)
    MoveArmWithStallTimeDetection(back_arm, 200, 90, 500)
    MoveBackward(50)
    MoveArmWithStallTimeDetection(back_arm, -160, 90, 500)
    MoveForward(95)
    MoveBackward(70)
    drive_base.turn(-35)
    MoveArmWithStallTimeDetection(back_arm, 160, 90, 500)
    MoveBackward(100)
    MoveArmWithStallTimeDetection(back_arm, -200, 90, 500)
    MoveForward(160)
    MoveBackward(350)
#navigate to white and green block drop off area
StopAtWhite(-200)
MoveBackward(150)
drive_base.turn(-90)
SetSpeed(300)
SetGyro(False)
MoveBackward(350)
SetGyro(True)
drive_base.stop()
SetSpeed(300)
run_task(DriveForwardAndMoveArm(300, back_arm, 100, 90))
StopAtWhite(300)
SetSpeed(600)
if(greenAtFront == 1):
    MoveForward(450)
else:
    MoveForward(435)
drive_base.brake()
SetSpeed(200)
drive_base.turn(90)
SetSpeed(200)

#drop off white and green blocks
if(greenAtFront == 1):
    MoveForward(420)
    MoveArmWithStallTimeDetection(front_arm, -200, 70, 500)
    MoveBackward(80)
    MoveArmWithStallTimeDetection(front_arm, 200, 20, 500)
    MoveForward(10)
    MoveArmWithStallTimeDetection(front_arm, 200, 50, 500)
    MoveForward(130)
    MoveBackward(220)
    drive_base.turn(-19)
    MoveForward(150)
    MoveArmWithStallTimeDetection(front_arm, -200, 70, 500)
    MoveBackward(130)
    MoveArmWithStallTimeDetection(front_arm, 200, 70, 500)
    MoveForward(220)
    MoveBackward(90)
    drive_base.turn(19)
else:
    MoveForward(350)
    drive_base.turn(-29)
    MoveForward(50)
    MoveArmWithStallTimeDetection(front_arm, -200, 70, 500)
    MoveBackward(60)
    MoveArmWithStallTimeDetection(front_arm, 140, 70, 500)
    MoveForward(160)
    MoveBackward(135)
    drive_base.turn(27)
    MoveForward(140)
    MoveArmWithStallTimeDetection(front_arm, -140, 70, 500)
    MoveBackward(120)
    MoveArmWithStallTimeDetection(front_arm, 200, 70, 500)
    MoveForward(95)
    MoveBackward(90)

#discarded (very last part)
MoveBackward(330)
drive_base.turn(90)
SetGyro(False)
MoveBackward(250)
SetGyro(True)
canBreak = False
if(canBreak):
    run_task(DriveForwardAndMoveArm(550, front_arm, 200, -70))
    drive_base.turn(-90)
else:
    run_task(DriveForwardAndMoveArm(560, front_arm, 200, -70))
    drive_base.turn(-92)
SetSpeed(250)
MoveForward(300)
drive_base.turn(-3)
MoveForward(320)
left_motor.run_angle(500, 80)
wait(5000)

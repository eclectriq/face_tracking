import datetime
import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

def init(channel, angle = 0, minAngle = 0, maxAngle = 180, sweepStep = 0.5, sweepStepTracking = 0.1, sweepSleep = 0.01):
    servo = kit.servo[channel]
    servoData = {}
    angle = min(maxAngle, max(angle, minAngle))
    servoData['minAngle'] = minAngle
    servoData['maxAngle'] = maxAngle
    servoData['sweepStepPositive'] = servoData['sweepStepCurrent'] = sweepStep
    servoData['sweepStepNegative'] = -1 * sweepStep
    servoData['sweepStepTracking'] = sweepStepTracking
    servoData['sweepSleep'] = sweepSleep
    servoData['angle'] = angle
    servoData['targetCoordinates'] = [-1, -1]
    return (servo, servoData)

def rotateTo(servo, servoData, angle):
    if angle > servoData['maxAngle']:
        angle = servoData['maxAngle']
    elif angle < servoData['minAngle']:
        angle = servoData['minAngle']
    servoData['angle'] = angle
    servo.angle = angle
 
def pause(servo):
    return servo

def start(servo):
    return servo
    
def stop(servo):
    return servo
    
def sweep(servo, servoData):
    newServoAngle = servoData['angle'] + servoData['sweepStepCurrent']
    if(newServoAngle >= servoData['maxAngle']):
        newServoAngle = servoData['maxAngle']
        servoData['sweepStepCurrent'] = servoData['sweepStepNegative']
        time.sleep(1)
    if(newServoAngle <= servoData['minAngle']):
        newServoAngle = servoData['minAngle']
        servoData['sweepStepCurrent'] = servoData['sweepStepPositive']
        time.sleep(1)
    rotateTo(servo, servoData, newServoAngle)

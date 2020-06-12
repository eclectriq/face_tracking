import datetime
import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

def init(channel, angle = 0, minAngle = 0, maxAngle = 180, sweepStep = 0.5, sweepStepTracking = 0.1, sweepSleep = 0.01):
    servo = kit.servo[channel]
    servo_data = {}
    angle = min(maxAngle, max(angle, minAngle))
    servo_data['minAngle'] = minAngle
    servo_data['maxAngle'] = maxAngle
    servo_data['sweepStepPositive'] = servo_data['sweepStepCurrent'] = sweepStep
    servo_data['sweepStepNegative'] = -1 * sweepStep
    servo_data['sweepStepTracking'] = sweepStepTracking
    servo_data['sweepSleep'] = sweepSleep
    servo_data['angle'] = angle
    servo_data['targetCoordinates'] = [-1, -1]
    return (servo, servo_data)

def rotate_to(servo, servo_data, angle):
    if angle > servo_data['maxAngle']:
        angle = servo_data['maxAngle']
    elif angle < servo_data['minAngle']:
        angle = servo_data['minAngle']
    servo_data['angle'] = angle
    servo.angle = angle
    
def step_toward_target(servo, servo_data, target_coordinates, frameH, frameW):
    # TODO: Replace with vector math to calculate difference in location to center the target
    halfFrameW = frameW / 2
    targetX, targetY = target_coordinates
    servoStep = servo_data['sweepStepTracking']
    if halfFrameW - 25 <= targetX <= halfFrameW + 25:
        stopped = True
    elif targetX > halfFrameW + 25:
        rotate_to(servo, servo_data, servo_data['angle'] - servoStep)
    else:
        rotate_to(servo, servo_data, servo_data['angle'] + servoStep)
        servo_data['angle'] = servo_data['angle'] + servoStep
    if servo_data['angle'] > servo_data['maxAngle']:
        servo_data['angle'] = servo_data['maxAngle']
    elif servo_data['angle'] < servo_data['minAngle']:
        servo_data['angle'] = servo_data['minAngle']
    time.sleep(0.005) # need a sleep time in order to reduce the servo from overshooting
 
def pause(servo):
    return servo

def start(servo):
    return servo
    
def stop(servo):
    return servo
    
def sweep(servo, servo_data):
    newServoAngle = servo_data['angle'] + servo_data['sweepStepCurrent']
    if(newServoAngle >= servo_data['maxAngle']):
        newServoAngle = servo_data['maxAngle']
        servo_data['sweepStepCurrent'] = servo_data['sweepStepNegative']
        time.sleep(1)
    if(newServoAngle <= servo_data['minAngle']):
        newServoAngle = servo_data['minAngle']
        servo_data['sweepStepCurrent'] = servo_data['sweepStepPositive']
        time.sleep(1)
    rotate_to(servo, servo_data, newServoAngle)

import datetime
import time
import RPi.GPIO as GPIO

def init(pin, frequency, angle = 0, minDuty = 3, maxDuty = 12, minAngle = 0, maxAngle = 180, sweepStep = 1, sweepSleep = 1):
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)
    p = GPIO.PWM(pin, frequency)
    servoData = {}
    servoData['minDuty'] = minDuty
    servoData['maxDuty'] = maxDuty
    servoData['minAngle'] = minAngle
    servoData['maxAngle'] = maxAngle
    servoData['sweepStepPositive'] = servoData['sweepStepCurrent'] = sweepStep
    servoData['sweepStepNegative'] = -1 * sweepStep
    servoData['sweepSleep'] = sweepSleep
    servoData['angle'] = angle
    return (p, servoData)

def rotateTo(servo, servoData, angle):
    if angle > 180:
        angle = 180
    elif angle < 0:
        angle = 0
    dc = translate(angle, servoData['minAngle'], servoData['maxAngle'], servoData['minDuty'], servoData['maxDuty'])
    servoData['angle'] = angle
    servo.ChangeDutyCycle(dc)
    time.sleep(0.01) # TODO: Perhaps make this a parameter
    
def pause(servo):
    servo.ChangeDutyCycle(0)

def start(servo):
    servo.start(0)
    
def stop(servo):
    servo.stop()

def translate(value, fromLow, fromHigh, toLow, toHigh):
    return (toHigh-toLow) * (value-fromLow) / (fromHigh-fromLow) + toLow

def sweep(servo, servoData):
    newServoAngle = servoData['angle'] + servoData['sweepStepCurrent']
    if(newServoAngle >= servoData['maxAngle']):
        newServoAngle = servoData['maxAngle']
        servoData['sweepStepCurrent'] = servoData['sweepStepNegative']
        servo.ChangeDutyCycle(0)
        time.sleep(1)
    if(newServoAngle <= servoData['minAngle']):
        newServoAngle = servoData['minAngle']
        servoData['sweepStepCurrent'] = servoData['sweepStepPositive']
        servo.ChangeDutyCycle(0)
        time.sleep(1)
    rotateTo(servo, servoData, newServoAngle)
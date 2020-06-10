import datetime
import time
import RPi.GPIO as GPIO
import servo as servoControls

SERVO_PIN = 12
SERVO_PIN = 33
GPIO.setmode(GPIO.BOARD)

servo, servoData = servoControls.init(SERVO_PIN, 50)

def stepTowardTarget(servo, servoData, targetCoordinates, frameH, frameW):
    halfFrameW = frameW / 2
    targetX, targetY = targetCoordinates
    servoStep = servoData['sweepStepPositive']
    if halfFrameW - 25 <= targetX <= halfFrameW + 25:
        stopped = True
        print("Stopped at" + str(datetime.datetime.now()))
        time.sleep(0.1)
    elif targetX > halfFrameW + 25:
        servoControls.rotateTo(servo, servoData, servoData['angle'] - servoStep)
    else:
        servoControls.rotateTo(servo, servoData, servoData['angle'] + servoStep)
        servoData['angle'] = servoData['angle'] + servoStep
    if servoData['angle'] > 180:
        servoData['angle'] = 180
    elif servoData['angle'] < 0:
        servoData['angle'] = 0
    #print(servoData['angle'])
    
servoControls.start(servo)

while True:
    boxes = [] # TODO simulate face found
    if len(boxes) > 0:
        x0, y0, width, height = boxes[0]
        stepTowardTarget(servo, servoData, [round(abs((x0+(x0+width))/2)), round(abs((y0+(y0+height))/2))], frameH, frameW)
    else:
        servoControls.sweep(servo, servoData)


print('stopping...')
time.sleep(1)
servo.stop()
GPIO.cleanup()
exit()



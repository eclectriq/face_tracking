import cv2
import datetime
import time
import _thread
import RPi.GPIO as GPIO
import servoAdafruit as servoControls

RUNNING = True
SERVO_PIN = 40 #33 #12
LED_PIN = 17

# Load the model.
net = cv2.dnn_DetectionModel('face-detection-adas-0001.xml',
                            'face-detection-adas-0001.bin')
# Specify target device.
net.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)

# Read an image.
v_c = cv2.VideoCapture(0)

#GPIO.setmode(GPIO.BOARD)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.output(LED_PIN, GPIO.LOW)

servo, servoData = servoControls.init(0, sweep_tracking_step = 0.5) #SERVO_PIN, 50)

def ledOn():
    GPIO.output(LED_PIN, GPIO.HIGH)

def ledOff():
    GPIO.output(LED_PIN, GPIO.LOW)

def stepTowardTarget(servo, servoData, targetCoordinates, frameH, frameW):
    halfFrameW = frameW / 2
    targetX, targetY = targetCoordinates
    print(datetime.datetime.now())
    servoStep = servoData['sweepStepTracking']
    if halfFrameW - 25 <= targetX <= halfFrameW + 25:
        stopped = True
    elif targetX > halfFrameW + 25:
        servoControls.rotateTo(servo, servoData, servoData['angle'] - servoStep)
    else:
        servoControls.rotateTo(servo, servoData, servoData['angle'] + servoStep)
        servoData['angle'] = servoData['angle'] + servoStep
    if servoData['angle'] > 180:
        servoData['angle'] = 180
    elif servoData['angle'] < 0:
        servoData['angle'] = 0
    time.sleep(0.08)
    #print(servoData['angle'])
    
cv2.VideoCapture.set(v_c, cv2.CAP_PROP_FPS, 15)

# Read first frame to get window frame shape
_, frame = v_c.read()
if frame is None:
        raise Exception('Image not found!')
frameH, frameW, frameChannels = frame.shape


def control_servo():
    time.sleep(1)
    while RUNNING:
        if servoData['targetCoordinates'] != [-1,-1]:
            stepTowardTarget(servo, servoData, servoData['targetCoordinates'], frameH, frameW)
        else:
            servoControls.sweep(servo, servoData)
            time.sleep(0.01)

servoThread = _thread.start_new_thread(control_servo, ())


try:
    while True:
        _, frame = v_c.read()
        if frame is None:
                raise Exception('Image not found!')
        frame = cv2.flip(frame, flipCode=-1)
        # Perform an inference.
        _, confidences, boxes = net.detect(frame, confThreshold=0.5)
            
        # Draw detected faces on the frame.
        for confidence, box in zip(list(confidences), boxes):
            cv2.rectangle(frame, box, color=(0, 255, 0))
        
        if len(boxes) > 0:
            ledOn()
            x0, y0, width, height = boxes[0]
            servoData['targetCoordinates'] = [round(abs((x0+(x0+width))/2)), round(abs((y0+(y0+height))/2))]
            print('setting coordinates')
            #stepTowardTarget(servo, servoData, [round(abs((x0+(x0+width))/2)), round(abs((y0+(y0+height))/2))], frameH, frameW)
        else:
            ledOff()
            servoData['targetCoordinates'] = [-1, -1]
            #servoControls.sweep(servo, servoData)
            
        #frame_rate = int(cv2.VideoCapture.get(v_c, cv2.CAP_PROP_FPS))
        
        #cv2.putText(frame, str(frame_rate) + " FPS",
        #    (10, 20),
        #    cv2.FONT_HERSHEY_SIMPLEX,
        #    1, (255, 255, 255), 2)

        # Save the frame to an image file.
        cv2.imshow('Video', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    RUNNING = False
    print('stopping...')
    time.sleep(1)
    ledOff()
    #servo.stop()
    GPIO.cleanup()
    v_c.release()
    cv2.destroyAllWindows()
    exit()


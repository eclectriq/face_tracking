#

"""
Responsible for Pan actions based on servo-adafruit
"""
import servoAdafruit as Servo_Controls

def init(pan_data, servo_data):
	servo, servo_data = Servo_Controls.init()

def step_toward_target(servo, servo_data, target_coordinates, frameH, frameW):
    # TODO: Replace with vector math to calculate difference in location to center the target
    halfFrameW = frameW / 2
    targetX, targetY = target_coordinates
    servoStep = servo_data['sweep_tracking_step']
    if halfFrameW - 25 <= targetX <= halfFrameW + 25:
        stopped = True
    elif targetX > halfFrameW + 25:
        rotate_to(servo, servo_data, servo_data['angle'] - servoStep)
    else:
        rotate_to(servo, servo_data, servo_data['angle'] + servoStep)
        servo_data['angle'] = servo_data['angle'] + servoStep
    if servo_data['angle'] > servo_data['max_angle']:
        servo_data['angle'] = servo_data['max_angle']
    elif servo_data['angle'] < servo_data['min_angle']:
        servo_data['angle'] = servo_data['min_angle']
    time.sleep(0.005) # need a sleep time in order to reduce the servo from overshooting


def sweep_step(servo, servo_data, pan_data):
    new_servo_angle = servo_data['angle'] + servo_data['sweep_step_current']
    if(new_servo_angle >= servo_data['max_angle']):
        new_servo_angle = servo_data['max_angle']
        servo_data['sweepStepCurrent'] = servo_data['sweep_step_negative']
        time.sleep(pan_data['sweep_sleep_time'])
    if(new_servo_angle <= servo_data['min_angle']):
        new_servo_angle = servo_data['min_angle']
        servo_data['sweepStepCurrent'] = servo_data['sweepStepPositive']
        time.sleep(pan_data['sweep_sleep_time'])
    Servo_Controls.rotate_to(servo, servo_data, new_servo_angle)
import datetime
import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

def init(name, channel, angle = 0, min_angle = 0, max_angle = 180, sweep_step = 0.5, sweep_step = 0.1, sweep_sleep = 0.01):
    servo = kit.servo[channel]
    servo_data = {'name': name, 'channel': channel}
    angle = min(max_angle, max(angle, min_angle))
    servo_data['min_angle'] = min_angle
    servo_data['max_angle'] = max_angle
    servo_data['sweep_step_positive'] = servo_data['sweep_step_current'] = sweepStep
    servo_data['sweep_step_negative'] = -1 * sweepStep
    servo_data['sweep_step'] = sweep_step
    servo_data['sweep_sleep'] = sweep_sleep
    servo_data['angle'] = angle
    servo_data['targetCoordinates'] = [-1, -1]
    return (servo, servo_data)

def rotate_to(servo, servo_data, angle):
    if angle > servo_data['max_angle']:
        angle = servo_data['max_angle']
    elif angle < servo_data['min_angle']:
        angle = servo_data['min_angle']
    servo_data['angle'] = angle
    servo.angle = angle

def sweep_step(servo, servo_data):
    newServoAngle = servo_data['angle'] + servo_data['sweep_step_current']
    if(newServoAngle >= servo_data['max_angle']):
        newServoAngle = servo_data['max_angle']
        servo_data['sweep_step_current'] = servo_data['sweep_step_negative']
        time.sleep(servo_data['sweep_sleep'])
    if(newServoAngle <= servo_data['min_angle']):
        newServoAngle = servo_data['min_angle']
        servo_data['sweep_step_current'] = servo_data['sweep_step_positive']
        time.sleep(servo_data['sweep_sleep'])
    rotate_to(servo, servo_data, newServoAngle)

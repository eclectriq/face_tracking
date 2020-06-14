#

"""
Responsible for Tilt actions based on servo-adafruit
"""
import servoAdafruit as servo_controls
import time

def step_toward_target(servo, servo_data, target_coordinates, frame_h, frame_w):
    # TODO: Replace with vector math to calculate difference in location to center the target
    halfframe_h = frame_h / 2
    _, target_y = target_coordinates
    servo_step = servo_data['sweep_tracking_step']
    # TODO: Parameterize ???
    if halfframe_h - 25 <= target_y <= halfframe_h + 25:
        stopped = True
    elif target_y < halfframe_h + 25:
        servo_controls.rotate_to(servo, servo_data, servo_data['angle'] - servo_step)
    else:
        servo_controls.rotate_to(servo, servo_data, servo_data['angle'] + servo_step)
        servo_data['angle'] = servo_data['angle'] + servo_step
    if servo_data['angle'] > servo_data['max_angle']:
        servo_data['angle'] = servo_data['max_angle']
    elif servo_data['angle'] < servo_data['min_angle']:
        servo_data['angle'] = servo_data['min_angle']
    # need a sleep time in order to reduce the servo from overshooting
    time.sleep(servo_data['sweep_tracking_sleep'])
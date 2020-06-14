#

"""
Responsible for Tilt actions based on servo-adafruit
"""
import servoAdafruit as servo_controls

def step_toward_target(servo, servo_data, target_coordinates, frame_h, frame_w):
    # TODO: Replace with vector math to calculate difference in location to center the target
    halfframe_w = frame_w / 2
    target_x, _ = target_coordinates
    servoStep = servo_data['sweep_tracking_step']
    # TODO: Parameterize ???
    if halfframe_h - 25 <= target_x <= halfframe_w + 25:
        stopped = True
    elif target_x > halfframe_h + 25:
        rotate_to(servo, servo_data, servo_data['angle'] - servoStep)
    else:
        rotate_to(servo, servo_data, servo_data['angle'] + servoStep)
        servo_data['angle'] = servo_data['angle'] + servoStep
    if servo_data['angle'] > servo_data['max_angle']:
        servo_data['angle'] = servo_data['max_angle']
    elif servo_data['angle'] < servo_data['min_angle']:
        servo_data['angle'] = servo_data['min_angle']
    # need a sleep time in order to reduce the servo from overshooting
    time.sleep(servo_data['sweep_sleep'])

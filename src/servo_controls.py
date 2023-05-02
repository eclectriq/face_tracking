"""
Responsible for servo actions agnostic to the hardware driver
"""
import time


def init(servo_init_fn, name, channel=0, angle=0, min_angle=0, max_angle=180, sweep_step=0.5, sweep_step_sleep=0.01,
         sweep_tracking_step=0.1, sweep_sleep=0.01, sweep_complete_sleep=1, sweep_tracking_sleep=0.01,
         shutdown_angle=0):
    servo = servo_init_fn()
    servo_data = {'name': name, 'channel': channel}
    angle = min(max_angle, max(angle, min_angle))
    servo_data['min_angle'] = min_angle
    servo_data['max_angle'] = max_angle
    servo_data['sweep_step_positive'] = servo_data['sweep_step_current'] = sweep_step
    servo_data['sweep_step_negative'] = -1 * sweep_step
    servo_data['sweep_step'] = sweep_step
    servo_data['sweep_step_sleep'] = sweep_step_sleep
    servo_data['sweep_sleep'] = sweep_sleep
    servo_data['sweep_tracking_step'] = sweep_tracking_step
    servo_data['sweep_tracking_sleep'] = sweep_tracking_sleep
    servo_data['angle'] = angle
    servo_data['shutdown_angle'] = min(max_angle, max(shutdown_angle, min_angle))
    servo.angle = angle
    return servo, servo_data


def rotate_to(servo, servo_data, angle):
    """
    Request rotation to a given angle
    """
    if angle > servo_data['max_angle']:
        angle = servo_data['max_angle']
    elif angle < servo_data['min_angle']:
        angle = servo_data['min_angle']
    servo_data['angle'] = angle
    servo.angle = angle
    return servo, servo_data


def step(servo, servo_data):
    """
    Request one step in a given direction. This alternates direction after min or max angle is reached.
    """
    new_servo_angle = servo_data['angle'] + servo_data['sweep_step_current']
    if new_servo_angle >= servo_data['max_angle']:
        new_servo_angle = servo_data['max_angle']
        servo_data['sweep_step_current'] = servo_data['sweep_step_negative']
        time.sleep(servo_data['sweep_sleep'])
    if new_servo_angle <= servo_data['min_angle']:
        new_servo_angle = servo_data['min_angle']
        servo_data['sweep_step_current'] = servo_data['sweep_step_positive']
        time.sleep(servo_data['sweep_sleep'])
    rotate_to(servo, servo_data, new_servo_angle)
    time.sleep(servo_data['sweep_step_sleep'])
    return servo, servo_data


def step_toward_target(servo, servo_data, current, target, target_give=25):
    """
    Request one step towards the target given the current position.

    target_give allows for the algorithm to settle within a given range +/- 25 pixels. This helps to reduce jitter.
    """
    servo_step = servo_data['sweep_tracking_step']
    if target - target_give <= current <= target + target_give:
        stopped = True
    elif current < target + target_give:
        rotate_to(servo, servo_data, servo_data['angle'] - servo_step)
    else:
        rotate_to(servo, servo_data, servo_data['angle'] + servo_step)
    # need a sleep time in order to reduce the servo from overshooting
    time.sleep(servo_data['sweep_tracking_sleep'])
    return servo, servo_data

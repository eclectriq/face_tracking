"""
Responsible for Tilt actions based on servo-adafruit
"""
import servo_controls


def step_toward_target(servo, servo_data, target_coordinates, frame_h, frame_w):
    _, target_y = target_coordinates
    halfframe_h = frame_h / 2
    servo_controls.step_toward_target(servo, servo_data, target_y, halfframe_h)

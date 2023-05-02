"""
Responsible for Pan actions based on servo-adafruit
"""
import servo_controls


def step_toward_target(servo, servo_data, target_coordinates, frame_h, frame_w):
    target_x, _ = target_coordinates
    halfframe_w = frame_w / 2
    servo_controls.step_toward_target(servo, servo_data, target_x, halfframe_w)

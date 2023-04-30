import argparse
from enum import Enum
import yaml
import collections
import common
import numpy as np
import os
from PIL import Image
import imutils
import re
import servoAdafruit as servo_controls
import datetime
import time
import _thread
import RPi.GPIO as GPIO
from functools import partial

global SHARED_STATE
SHARED_STATE = {"running" : True}

def control_servo(servo, servo_data, target_data, target_fn, on_stop_fn = None):
    time.sleep(1)
    servo_controls.sweep_step(servo, servo_data)
    if on_stop_fn != None:
        print('Calling on_stop_fn')
        on_stop_fn(servo, servo_data)
    print('No longer running servo {}'.format(servo_data['name']))

Object = collections.namedtuple('Object', ['id', 'score', 'bbox'])
class TargetState(Enum):
    UNKNOWN = 1
    ACQUIRED = 2
    TRACKING = 3
    LOST = 4

def init_servo(name, channel):
    print('Initializing servo {} with config {}'.format(name, {}))
    servo, servo_data = servo_controls.init(name, channel=channel)
    return (servo, servo_data)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--channel', help='servo channel such as 0',
                        default=0)
    args = parser.parse_args()

    print('Initializing servo')
    servo, servo_data = init_servo('pan', int(args.channel))
    print('Servo initialized')

    for x in range(18):
        time.sleep(0.25)
        print('rotating to {}'.format(x * 10))
        servo_controls.rotate_to(servo, servo_data, x * 10)

    print('sweeping for 30 seconds')
    for x in range(3000):
        servo_controls.sweep_step(servo, servo_data)

    print('Shutting down')
    SHARED_STATE['running'] = False
    time.sleep(5)
    print('Goodybe')

if __name__ == '__main__':
    main()


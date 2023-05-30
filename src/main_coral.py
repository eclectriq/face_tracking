# Demonstration of face tracking with a camera and optional servo(s)
# Incorporates sound during particular events
import argparse
from enum import Enum
from sound_controls import play_sound
import pan_servo as pan_controls
import tilt_servo as tilt_controls
import servo_controls as servo_controls
import yaml
import collections
import cv2
import numpy as np
import os
from PIL import Image
import re
import datetime
import time
import _thread
import tpu
import servoAdafruit
from functools import partial

# Seed play_sound path so not to repeat it
play_sound = partial(play_sound, os.path.join("../media", "audio"))

global SHARED_STATE
SHARED_STATE = {"running": True}

Object = collections.namedtuple('Object', ['id', 'score', 'bbox'])


class TrackingState(Enum):
    UNKNOWN = 1
    NOT_TRACKING = 2
    TRACKING = 3
    LOST = 4


def main():
    default_model_dir = '../'
    default_model = 'mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite'
    default_labels = 'coco_labels.txt'
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', help='.tflite model path',
                        default=os.path.join(default_model_dir, default_model))
    parser.add_argument('--labels', help='label file path',
                        default=os.path.join(default_model_dir, default_labels))
    parser.add_argument('--top_k', type=int, default=3,
                        help='number of categories with highest score to display')
    args = parser.parse_args()

    with open('../config.yaml', 'r') as stream:
        try:
            print('loading configuration')
            config = yaml.safe_load(stream)
            # DEFAULTS
            config['threshold'] = config.get('threshold', 0.5)
            config['camera'] = config.get('camera', {})
            config['camera']['index'] = config.get('index', 0)
            config['camera']['orientation'] = config.get('orientation', 0)
        except yaml.YAMLError as exc:
            print('Unable to read YAML')
            print(exc)

    target_data = {'target_coordinates': [-1, -1]}

    print('Initializing servos')
    if 'pan' in config:
        print('Initializing pan servo')
        pan_servo, pan_servo_data = init_servo('pan', config['pan'])
        pan_servo_thread = _thread.start_new_thread(control_servo,
                                                    (pan_servo, pan_servo_data, target_data,
                                                     pan_controls.step_toward_target,
                                                     lambda servo, servo_data:
                                                     servo_controls.rotate_to(servo, servo_data,
                                                                              servo_data['shutdown_angle'])))
    if 'tilt' in config:
        print('Initializing tilt servo')
        tilt_servo, tilt_servo_data = init_servo('tilt', config['tilt'])
        tilt_servo_thread = _thread.start_new_thread(control_servo,
                                                     (tilt_servo, tilt_servo_data, target_data,
                                                      tilt_controls.step_toward_target,
                                                      lambda servo, servo_data: servo_controls.rotate_to(servo,
                                                                                                         servo_data,
                                                                                                         servo_data[
                                                                                                             'shutdown_angle'])))
    print('Servos initialized')

    print('Loading {} with {} labels.'.format(args.model, args.labels))
    interpreter = tpu.make_interpreter(args.model)
    interpreter.allocate_tensors()
    labels = load_labels(args.labels)
    print('labels loaded')

    #    cv2.VideoCapture.set(v_c, cv2.CAP_PROP_FPS, 15)

    print('Capturing first frame for shape')
    cap = cv2.VideoCapture(config['camera']['index'])
    # Read first frame to get window frame shape
    _, frame = cap.read()
    if frame is None:
        raise Exception('Image not found!')

    last_target_lost = None
    last_target_lost_time = datetime.datetime.now()
    tracking_state = TrackingState.UNKNOWN

    play_sound(config['sounds']['searching'])

    print('Starting video loop')

    tracking_state = TrackingState.NOT_TRACKING

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        # TODO: Use config orientation
        # frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)

        # frame = imutils.rotate(frame, 90)
        h, w, layers = frame.shape
        # aspect_ratio = w / h
        # print('{} {} {}'.format(h, w, aspect_ratio))
        cv2_im = frame  # cv2.resize(frame, (1920, 1080))

        cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
        pil_im = Image.fromarray(cv2_im_rgb)

        tpu.set_input(interpreter, pil_im)
        interpreter.invoke()
        objs = get_output(interpreter, score_threshold=config['threshold'], top_k=args.top_k)
        cv2_im = append_objs_to_img(cv2_im, objs, labels)

        face = next(filter(lambda a: a.id == 0, objs), None)

        if face is not None:
            if tracking_state == TrackingState.NOT_TRACKING:
                play_sound(config['sounds']['target_acquired'])
                tracking_state = TrackingState.TRACKING
            elif tracking_state == TrackingState.LOST:
                tracking_state = TrackingState.TRACKING

            height, width, channels = cv2_im.shape

            x0, y0, x1, y1 = list(face.bbox)
            x0, y0, x1, y1 = int(x0 * width), int(y0 * height), int(x1 * width), int(y1 * height)

            last_target_lost = 0
            # Set target coordinates based on mid-point of the axis of bounding box
            target_data['target_coordinates'] = [round(abs((x0 + (x1)) / 2)), round(abs((y0 + (y1)) / 2))]
        else:
            # target may have been lost
            if tracking_state == TrackingState.TRACKING:
                tracking_state = TrackingState.LOST
                # track lost time
                last_target_lost_time = datetime.datetime.now()
            if tracking_state == TrackingState.LOST and (
                    last_target_lost_time is None or (datetime.datetime.now() - last_target_lost_time).seconds > 10):
                # if lost for over a second, reset tracking_state back to default
                target_data['target_coordinates'] = [-1, -1]
                play_sound(config['sounds']['target_lost'])
                tracking_state = TrackingState.UNKNOWN
                last_target_lost_time = None

        if config['camera']['display']:
            cv2.imshow('frame', cv2_im)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print('Shutting down')
    SHARED_STATE['running'] = False
    time.sleep(5)
    cap.release()
    cv2.destroyAllWindows()
    print('Goodbye')


def append_objs_to_img(cv2_im, objs, labels):
    height, width, channels = cv2_im.shape
    for obj in objs:
        x0, y0, x1, y1 = list(obj.bbox)
        x0, y0, x1, y1 = int(x0 * width), int(y0 * height), int(x1 * width), int(y1 * height)
        percent = int(100 * obj.score)
        label = '{}% {}'.format(percent, labels.get(obj.id, obj.id))

        cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2_im = cv2.putText(cv2_im, label, (x0, y0 + 30),
                             cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
    return cv2_im


def control_servo(servo, servo_data, target_data, target_fn, on_stop_fn=None):
    time.sleep(1)
    while SHARED_STATE['running']:
        if target_data['target_coordinates'] != [-1, -1]:
            target_fn(servo, servo_data, target_data['target_coordinates'], 480, 640)  # TODO: Read from cv2
        else:
            servo_controls.step(servo, servo_data)
    if on_stop_fn is not None:
        print('Calling on_stop_fn')
        on_stop_fn(servo, servo_data)
    print('No longer running servo {}'.format(servo_data['name']))


def load_labels(path):
    p = re.compile(r'\s*(\d+)(.+)')
    with open(path, 'r', encoding='utf-8') as f:
        lines = (p.match(line).groups() for line in f.readlines())
        return {int(num): text.strip() for num, text in lines}


class BBox(collections.namedtuple('BBox', ['xmin', 'ymin', 'xmax', 'ymax'])):
    """Bounding box.
    Represents a rectangle which sides are either vertical or horizontal, parallel
    to the x or y axis.
    """
    __slots__ = ()


def get_output(interpreter, score_threshold, top_k, image_scale=1.0):
    """Returns list of detected objects."""
    boxes = tpu.output_tensor(interpreter, 0)
    class_ids = tpu.output_tensor(interpreter, 1)
    scores = tpu.output_tensor(interpreter, 2)
    count = int(tpu.output_tensor(interpreter, 3))

    def make(i):
        ymin, xmin, ymax, xmax = boxes[i]
        return Object(
            id=int(class_ids[i]),
            score=scores[i],
            bbox=BBox(xmin=np.maximum(0.0, xmin),
                      ymin=np.maximum(0.0, ymin),
                      xmax=np.minimum(1.0, xmax),
                      ymax=np.minimum(1.0, ymax)))

    return [make(i) for i in range(top_k) if scores[i] >= score_threshold]


def init_servo(name, config):
    print('Initializing servo {} with config {}'.format(name, config))
    servo, servo_data = servo_controls.init(lambda: servoAdafruit.init(config['channel']), name, **config)
    return servo, servo_data


if __name__ == '__main__':
    main()

# Copyright 2019 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""A demo that runs object detection on camera frames using OpenCV.

TEST_DATA=../all_models

Run face detection model:
python3 detect.py \
  --model ${TEST_DATA}/mobilenet_ssd_v2_face_quant_postprocess_edgetpu.tflite

Run coco model:
python3 detect.py \
  --model ${TEST_DATA}/mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite \
  --labels ${TEST_DATA}/coco_labels.txt

"""
import argparse
from enum import Enum
import collections
import common
import cv2
import numpy as np
import os
from PIL import Image
import imutils
import re
import tflite_runtime.interpreter as tflite
import servoAdafruit as ServoControls
import datetime
import time
import _thread
import RPi.GPIO as GPIO

TARGET_ACQUIRED = 'target-acquired.mp3'

sound_last_play = {}

def initCameras(cameras_config):
    # accounting for the number of threads and cameras_config, setup each camera
    # camera setup uses its camera_config to determine if it supports, pan and/or tilt (or neither)
    # pan and tilt depend on a face detected and must move to center the face


def play_sound(s):
    if s not in sound_last_play or (s in sound_last_play and (datetime.datetime.now() - sound_last_play[s]).seconds > 1):
        os.system('nohup nvlc ' + s + ' --play-and-exit > /dev/null 2>&1 &')
        sound_last_play[s] = datetime.datetime.now()

global RUNNING
RUNNING = True
SERVO_PIN = 40 #33 #12
LED_PIN = 17

GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.output(LED_PIN, GPIO.LOW)

def ledOn():
    GPIO.output(LED_PIN, GPIO.HIGH)

def ledOff():
    GPIO.output(LED_PIN, GPIO.LOW)

def control_servo(servo, servoData):
    time.sleep(1)
    while RUNNING:
        if servoData['targetCoordinates'] != [-1,-1]:
            ServoControls.step_toward_target(servo, servoData, servoData['targetCoordinates'], 480, 640) #TODO: Read from cv2
        else:
            ServoControls.sweep(servo, servoData)
            time.sleep(0.01)

Object = collections.namedtuple('Object', ['id', 'score', 'bbox'])
class TargetState(Enum):
    UNKNOWN = 1
    ACQUIRED = 2
    TRACKING = 3
    LOST = 4

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
    boxes = common.output_tensor(interpreter, 0)
    class_ids = common.output_tensor(interpreter, 1)
    scores = common.output_tensor(interpreter, 2)
    count = int(common.output_tensor(interpreter, 3))

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

def main():
    default_model_dir = './' #'../all_models'
    default_model = 'mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite'
    default_labels = 'coco_labels.txt'
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', help='.tflite model path',
                        default=os.path.join(default_model_dir,default_model))
    parser.add_argument('--labels', help='label file path',
                        default=os.path.join(default_model_dir, default_labels))
    parser.add_argument('--top_k', type=int, default=3,
                        help='number of categories with highest score to display')
    parser.add_argument('--camera_idx', help='Index of which video source to use. ', default = 0)
    parser.add_argument('--threshold', type=float, default=0.5,
                        help='classifier score threshold')
    parser.add_argument('--min_angle', type=float, default=0.0, help='minimum angle for sweep')
    parser.add_argument('--max_angle', type=float, default=180.0, help='maximum angle for sweep')
    
    args = parser.parse_args()
    
    print('Initializing servo')
    servo, servoData = ServoControls.init(0, sweepStepTracking = 0.1, minAngle = args.min_angle, maxAngle = args.max_angle) #SERVO_PIN, 50)
    servoThread = _thread.start_new_thread(control_servo, (servo,servoData))

    print('Loading {} with {} labels.'.format(args.model, args.labels))
    interpreter = common.make_interpreter(args.model)
    interpreter.allocate_tensors()
    labels = load_labels(args.labels)
    
#    cv2.VideoCapture.set(v_c, cv2.CAP_PROP_FPS, 15)

    cap = cv2.VideoCapture(args.camera_idx)
    # Read first frame to get window frame shape
    _, frame = cap.read()
    if frame is None:
            raise Exception('Image not found!')
    frameH, frameW, frameChannels = frame.shape
    
    lastTargetLost = None
    lastTargetLostTime = datetime.datetime.now()
    targetState = TargetState.UNKNOWN
    
    play_sound('searching.mp3')

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        #frame = cv2.flip(frame, flipCode=1)
        frame = imutils.rotate(frame, 90)
        h, w, layers = frame.shape
        aspect_ratio = w / h    
        cv2_im = frame # cv2.resize(frame, (1920, 1080))

        cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
        pil_im = Image.fromarray(cv2_im_rgb)

        common.set_input(interpreter, pil_im)
        interpreter.invoke()
        objs = get_output(interpreter, score_threshold=args.threshold, top_k=args.top_k)
        cv2_im = append_objs_to_img(cv2_im, objs, labels)
        
        face = next(filter(lambda a: a.id == 0, objs), None)
        
        if face != None:
            if targetState == TargetState.UNKNOWN:
                targetState = TargetState.ACQUIRED
            height, width, channels = cv2_im.shape
            
            x0, y0, x1, y1 = list(face.bbox)
            x0, y0, x1, y1 = int(x0*width), int(y0*height), int(x1*width), int(y1*height)
    
            lastTargetLost = 0
            servoData['targetCoordinates'] = [round(abs((x0+(x1))/2)), round(abs((y0+(y1))/2))]
        else:
            # target may have been lost
            if targetState == TargetState.TRACKING:
                targetState = TargetState.LOST
                # track lost time
                lastTargetLostTime = datetime.datetime.now()
            if targetState == TargetState.LOST and (lastTargetLostTime == None or (datetime.datetime.now() - lastTargetLostTime).seconds > 2):
                # if lost for over a second, reset targetState back to default
                servoData['targetCoordinates'] = [-1,-1]
                play_sound('are-still-there.mp3')
                targetState = TargetState.UNKNOWN
                lastTargetLostTime = None
            
        if targetState == TargetState.ACQUIRED:
            play_sound(TARGET_ACQUIRED)
            targetState = TargetState.TRACKING
        
        cv2.imshow('frame', cv2_im)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    RUNNING = False
    cap.release()
    cv2.destroyAllWindows()

def append_objs_to_img(cv2_im, objs, labels):
    height, width, channels = cv2_im.shape
    for obj in objs:
        x0, y0, x1, y1 = list(obj.bbox)
        x0, y0, x1, y1 = int(x0*width), int(y0*height), int(x1*width), int(y1*height)
        percent = int(100 * obj.score)
        label = '{}% {}'.format(percent, labels.get(obj.id, obj.id))

        cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2_im = cv2.putText(cv2_im, label, (x0, y0+30),
                             cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
    return cv2_im

if __name__ == '__main__':
    main()


import cv2
import datetime
import time

# Load the model.
net = cv2.dnn_DetectionModel('face-detection-adas-0001.xml',
                            'face-detection-adas-0001.bin')
# Specify target device.
net.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)

# Read an image.
v_c = cv2.VideoCapture(0)

cv2.VideoCapture.set(v_c, cv2.CAP_PROP_FPS, 15)

# Read first frame to get window frame shape
_, frame = v_c.read()
if frame is None:
        raise Exception('Image not found!')
frameH, frameW, frameChannels = frame.shape

while True:
    _, frame = v_c.read()
    if frame is None:
            raise Exception('Image not found!')
    frame = cv2.flip(frame, flipCode=-1)
    # Perform an inference.
    _, confidences, boxes = net.detect(frame, confThreshold=0.5)
        
    # Draw detected faces on the frame.
    for confidence, box in zip(list(confidences), boxes):
        cv2.rectangle(frame, box, color=(0, 255, 0))
            
    #frame_rate = int(cv2.VideoCapture.get(v_c, cv2.CAP_PROP_FPS))
    
    #cv2.putText(frame, str(frame_rate) + " FPS",
    #    (10, 20),
    #    cv2.FONT_HERSHEY_SIMPLEX,
    #    1, (255, 255, 255), 2)

    # Save the frame to an image file.
    cv2.imshow('Video', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

print('stopping...')
time.sleep(1)
v_c.release()
cv2.destroyAllWindows()
exit()



from adafruit_servokit import ServoKit

# BEGIN Device specific setup
kit = ServoKit(channels=16)
# END device specific setup

def init(channel=0):
    return kit.servo[channel]

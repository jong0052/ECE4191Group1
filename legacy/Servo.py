from gpiozero import AngularServo
from time import sleep
import math

servo = AngularServo(18, min_pulse_width=0.0006, max_pulse_width=0.0023)

def openServo():
    servo.angle = -50
    
def closeServo():
    servo.angle = 50

while True:
    servo.min()
    sleep(5)
    servo.max()
    sleep(5)
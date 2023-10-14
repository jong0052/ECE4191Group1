import RPi.GPIO as GPIO
import time

class Servo:
    def __init__(self):
        
        self.control = [5, 5.5, 6, 6.5, 7, 7.5, 8, 8.5, 9, 9.5, 10]

        self.servo = 18

        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.servo, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo, 50)  # 50hz frequency
        
    def unload(self):

        self.pwm.start(7.5)  # starting duty cycle ( it set the servo to 0 degree )

        try:
            
            print("loop")
            # Rotate the servo in one direction (0 degrees)
            self.pwm.ChangeDutyCycle(8)
            time.sleep(1)
            self.pwm.ChangeDutyCycle(0)
            time.sleep(4)

            print("2")
            # Rotate the servo in the other direction (180 degrees)
            self.pwm.ChangeDutyCycle(4)
            time.sleep(2)
            self.pwm.ChangeDutyCycle(0)
            time.sleep(4)

        except KeyboardInterrupt:
            GPIO.cleanup()
            

        # Stop the PWM and cleanup
        self.pwm.stop()
        GPIO.cleanup()
        
    def moving(self):
        self.pwm.start(7.5)
        try:
            self.pwm.ChangeDutyCycle(4)
        except KeyboardInterrupt:
            GPIO.cleanup()
        


newServo = Servo()
newServo.moving()
time.sleep(5)
newServo.unload()
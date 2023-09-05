import numpy as np
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO_TRIGGER3 = 5
GPIO_ECHO3 = 6

GPIO.setup(GPIO_TRIGGER3, GPIO.OUT)
GPIO.setup(GPIO_ECHO3, GPIO.IN)

def distance(trig, echo):
        # set Trigger to HIGH
        
        
        GPIO.output(trig, True)

            # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(trig, False)

        StartTime = time.time()
        StopTime = time.time()
        timeout = time.time()
        timeout_threshold = 0.2

            # save StartTime
        while GPIO.input(echo) == 0 and (time.time()- timeout) < 0.2:
            StartTime = time.time()
                # print("start")

            # save time of arrival
        while GPIO.input(echo) == 1 and (time.time() - timeout) < 0.2:
            StopTime = time.time()
                # print("stop")

            # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
            # multiply with the sonic speed (34300 cm/s)
            # and divide by 2, because there and back
        dist = (TimeElapsed * 34300) / 2
        
        return dist

def multi_dist (num_samp = 10):
    dist_vec = []
    print_vec = []

    i = 0
    for i in range(5):
        for i in range (num_samp):
            dist = distance(GPIO_TRIGGER3, GPIO_ECHO3)
            if (dist >= 5 and dist <=40):
                dist_vec.append(dist)
                i+=1
        mean_dist = np.mean(dist_vec)
        print_vec.append(mean_dist)
    result = np.median(print_vec)
    return result

print(multi_dist())

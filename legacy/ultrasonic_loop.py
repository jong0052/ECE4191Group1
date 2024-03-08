import RPi.GPIO as GPIO
import time

from main import MPManager

#start Luke's code
# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# set GPIO Pins
GPIO_TRIGGER1 = 23 # Right
GPIO_ECHO1 = 24
GPIO_TRIGGER2 = 17 # Left
GPIO_ECHO2 = 27 # Left
GPIO_TRIGGER3 = 5 # Middle
GPIO_ECHO3 = 6 # Middle

ECHO2_arr = [17, 5, 23]
TRIG_arr = [27, 6, 24]
# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER1, GPIO.OUT)
GPIO.setup(GPIO_ECHO1, GPIO.IN)
GPIO.setup(GPIO_TRIGGER2, GPIO.OUT)
GPIO.setup(GPIO_ECHO2, GPIO.IN)
GPIO.setup(GPIO_TRIGGER3, GPIO.OUT)
GPIO.setup(GPIO_ECHO3, GPIO.IN)

def distance(trig, echo, pin = 1):
        # set Trigger to HIGH
        
        GPIO.output(trig, True)

            # set Trigger after 0.01ms to LOW
        if (pin == 1):
            time.sleep(0.00001)
        elif (pin == 2):
            time.sleep(0.000005)
        else:
            time.sleep(0.000006)
        GPIO.output(trig, False)

        StartTime = time.time()
        StopTime = time.time()
        timeout = time.time()
        timeout_threshold = 0.01

            # save StartTime
        while GPIO.input(echo) == 0 and (time.time()- timeout) < timeout_threshold:
            StartTime = time.time()
                # print("start")

            # save time of arrival
        while GPIO.input(echo) == 1 and (time.time() - timeout) < timeout_threshold:
            StopTime = time.time()
                # print("stop")

            # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
            # multiply with the sonic speed (34300 cm/s)
            # and divide by 2, because there and back
        dist = (TimeElapsed * 34300) / 2

        return dist

def multi_dist (num_samp = 3, trig = GPIO_TRIGGER3, echo = GPIO_ECHO3, pin =  1):
    dist_vec = []
    print_vec = []

    for i in range(5):
        for i in range (num_samp):
            dist = distance(trig , echo, pin)
            if (dist >= 5 and dist <=50):
                dist_vec.append(dist)
            else:
                dist_vec.append(200)
        mean_dist = np.mean(dist_vec)
        print_vec.append(mean_dist)
    result = np.median(print_vec)
    return result

def usLoop(manager_mp: MPManager):
    while True:
        if not (manager_mp.ready):
            time.sleep(0.5)
            continue

        value = []
        value.append(multi_dist(3, GPIO_TRIGGER2, GPIO_ECHO2,  1) / 100) 
        value.append(multi_dist(3, GPIO_TRIGGER3, GPIO_ECHO3,  2) / 100)
        value.append(multi_dist(3, GPIO_TRIGGER1, GPIO_ECHO1,  3) / 100)
        print("Left:" + str(value[2])+ " Middle:" + str(value[1]) + "Right" + str(value[0]))
 
            # print(value)
        #if (value[1] > 0.05 and value[1] < 0.50):
        manager_mp.usLeft_value.value = value[2] + 0.11
        manager_mp.usFront_value.value = value[1] + 0.12
        manager_mp.usRight_value.value = value[0] + 0.12
        manager_mp.usLeft_update.value = 1
        manager_mp.usFront_update.value = 1
        manager_mp.usRight_update.value = 1
        print("accepted")
        #else:
         #   usFront_value.value = 2
          #  usLeft_value.value = 2
           # usRight_value.value = 2
           # usLeft_update.value = 1
           # usFront_update.value = 1
           # usRight_update.value = 1
           # print("rejected")
    
        # print(usFront_value.value)
        time.sleep(0.01)
    #   usLeft_value.value = distance(GPIO_TRIGGER1, GPIO_ECHO1)
    #   usRight_value.value = distance(GPIO_TRIGGER2, GPIO_ECHO2)

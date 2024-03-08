import math
import numpy as np
import matplotlib as plt
from pykalman import KalmanFilter
import time



#imu has just two velocities
#wheel encoders have (x,y) and velocities


def update(mean1,var1,mean2,var2):
    new_mean = (var2*mean1 + var1*mean2)/(var1+var2)
    new_var = 1/(1/var2+1/var1)
    return [new_mean,new_var]
def predict(mean1,var1,mean2,var2):
    new_mean = mean1+mean2
    new_var = var1+var2
    return [new_mean,new_var]

imu = [0]
nav = [0]
angle = []

#while a new value comes in
while True:
    newImu = float(input('Imu reading: '))
    newNav = float(input('Nav reading: '))

    imu.append(newImu)
    nav.append(newNav)
    angle = (0.98)*(imu[-1]) + 0.02*nav[-1]
    angle_old = (0.98)*(imu[-2]) + 0.02*nav[-2]
    var1 = np.var(imu)
    var2 = np.var(nav)
    mean1 = np.mean(imu)
    mean2 = np.mean(nav)

    update_array = update(mean1,var1,mean2,var2)
    predict_array = predict(update_array[0],update_array[1],mean2,var2)

    print("update array" + str(update_array))
    print("Predict array" + str(predict_array))

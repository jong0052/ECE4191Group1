from utils.Obstacle import *

obstacles = [Circle(0,0.2,0.15)]
plotting = True
simulation = True

CAR_W = 0.238  # width of car
CAR_LF = 0.12  # distance from rear to vehicle front end
CAR_LB = 0.12  # distance from rear to vehicle back end

# vehicle rectangle vertices
CAR_VRX = [CAR_LF, CAR_LF, -CAR_LB, -CAR_LB, CAR_LF]
CAR_VRY = [CAR_W / 2, -CAR_W / 2, -CAR_W / 2, CAR_W / 2, CAR_W / 2]
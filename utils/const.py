from utils.Obstacle import *
import math

# obstacles = [Circle(0,0.2,0.15)]
obstacles = []
plotting = True
simulation = True

CAR_W = 0.238  # width of car
CAR_LF = 0.12  # distance from rear to vehicle front end
CAR_LB = 0.12  # distance from rear to vehicle back end

OBSTACLE_SIZE = math.sqrt(max(CAR_LB, CAR_LF)**2 + (CAR_W/2)**2)
MAX_PATH_ATTEMPTS = 10

# vehicle rectangle vertices
CAR_VRX = [CAR_LF, CAR_LF, -CAR_LB, -CAR_LB, CAR_LF]
CAR_VRY = [CAR_W / 2, -CAR_W / 2, -CAR_W / 2, CAR_W / 2, CAR_W / 2]
#ur5_config.py

import math
MAX_JOINTS = [2*math.pi, 2*math.pi, 2*math.pi, 2*math.pi, 2*math.pi, 2*math.pi]#, 1]
MIN_JOINTS = [-2*math.pi, -2*math.pi, -2*math.pi, -2*math.pi, -2*math.pi, -2*math.pi]#, 0]
MAX_VEL = [math.pi, math.pi, math.pi, math.pi, math.pi, math.pi]#, 1]
MIN_VEL = [-math.pi, -math.pi, -math.pi, -math.pi, -math.pi, -math.pi]#, -1]

#number of elements in a configuration
NUM_JOINTS=6

CONTROL_RATE = 250

PROTECTIVE_STOP_MASK = 0b100

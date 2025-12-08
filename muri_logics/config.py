import math
from types import SimpleNamespace

###################### Camera Config Values ########################

# Marker-Größen je nach ID (in mm)
MARKER_SIZES = {
    0: 175,   
    69: 63    
}
# Standardauflösung 640x480 V2
CAMERA_MATRIX_RAW = [
    [649.91775597, 0.0, 277.82553764],
    [0.0, 657.56547185, 196.4845238],
    [0.0, 0.0, 1.0]
]

DISTANCE_COEFFICIENT = [
    [-0.07703196],
    [-0.06672068],
    [-0.0093926],
    [-0.00615253],
    [0.18372124]
]

###################### MAX/Tolerance Values ########################

ANGLE_TOLLERANCE_INIT = 0.05
ANGLE_TOLLERANCE_TURN = 0.1
ANGLE_TOLLERANCE_DRIVE = 0.01
ANGLE_TOLLERANCE_FOLLOW = 0.01
MAX_ANGLE_VELOCITY_DRIVE = 0.4
MAX_ANGLE_VELOCITY_TURN_INIT = 0.2
MAX_ANGLE_VELOCITY_FOLLOW = 0.4
MAX_ANGLE = math.pi
ORIANTATION_DISTANCE = 0.5


MAX_VELOCITY = 0.08

GOAL_DISTANCE = 0.4

######################### Regulator Valus ###########################

KP_INIT = 0.2
KP_DRIVE = 2.0
KP_TURN = 0.2
KP_FOLLOW_ANGULAR = 0.2
KP_FOLLOW_LINEAR = 0.2

###################### Termination conditions #######################

STOP = SimpleNamespace(lvx = 0, lvy = 0, avz = 0, dr = None)
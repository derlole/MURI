import math
from types import SimpleNamespace

###################### Camera Config Values ########################

MARKER_SIZES = {
    0: 175,   
    69: 75
}

CAMERA_MATRIX_RAW = [
    [649.91775597, 0.0, 320.82553764],
    [0.0, 657.56547185, 240.4845238],
    [0.0, 0.0, 1.0]
]

DISTANCE_COEFFICIENT = [
    [0.047952517071021335],
    [-0.1370980321448537],
    [-0.023278788808772627],
    [-0.025988263120707157],
    [2.1353969013140452]
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

DEFAULT_VELOCITY = 0.0
MAX_VELOCITY = 0.2

GOAL_DISTANCE = 0.5
ORIANTATION_DISTANCE = 0.5

MINIMAL_SPEED_TO_SET = 0.0
MAXIMAL_SPEED_TO_SET = 0.2

MINIMAL_DISTANCE_FOLLOW = 0.2
MAXIMAL_DISTANCE_FOLLOW = 1.0

######################### Regulator Valus ###########################

KP_INIT = 0.2
KP_DRIVE = 2.0
KP_TURN = 2.0
KP_FOLLOW_ANGULAR = 2.0
KP_FOLLOW_LINEAR = 1.1

###################### Termination conditions #######################

STOP = SimpleNamespace(lvx = 0, lvy = 0, avz = 0, dr = None)

NULL_TUPEL = (0.0, 0.0, 0.0, 0.0)
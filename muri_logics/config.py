import math

###################### Camera Config Values ########################

MARKER_SIZE = 175
# Standardauflösung 640x480
# Standardauflösung 640x480 V2
CAMERA_MATRIX_RAW = [
    [593.91399635, 0.0, 334.37556895],
    [0.0, 586.86697636, 203.66210936],
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
MAX_ANGLE_VELOCITY_DRIVE = 0.4
MAX_ANGLE_VELOCITY_TURN_INIT = 0.2
MAX_ANGLE = math.pi
ORIANTATION_DISTANCE = 0.5


MAX_VELOCITY = 0.08

GOAL_DISTANCE = 0.25

######################### Regulator Valus ##########################

KP_INIT = 0.2
KP_DRIVE = 2.0
KP_TURN = 0.2

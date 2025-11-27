import math

###################### Camera Config Values ########################

MARKER_SIZE = 175
# Standardauflösung 640x480
CAMERA_MATRIX_RAW = [
    [786.247239319811, 0.0, 337.0469208743841],
    [0.0, 1437.603040586774, 239.58179870162698],
    [0.0, 0.0, 1.0]
]
DISTANCE_COEFFICIENT = [
    [-0.3882418324927825],
    [ 8.72452663009401],
    [ 0.009565477179223446],
    [ 0.016198197600272832],
    [-50.50814936392801]
]

###################### MAX/Tolerance Values ########################

ANGLE_TOLLERANCE_INIT = 0.05
ANGLE_TOLLERANCE_TURN = 0.1
ANGLE_TOLLERANCE_DRIVE = 0.01
MAX_ANGLE_VELOCITY_DRIVE = 0.4
MAX_ANGLE_VELOCITY_TURN_INIT = 0.2
MAX_ANGLE = math.pi

MAX_VELOCITY = 0.08

GOAL_DISTANCE = 0.25

######################### Regulator Valus ##########################

KP_INIT = 0.2
KP_DRIVE = 2.0
KP_TURN = 0.2

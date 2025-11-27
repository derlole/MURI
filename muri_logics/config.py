import math

###################### Camera Config Values ########################

MARKER_SIZE = 175
# Standardauflösung 640x480
# Standardauflösung 640x480 V2
CAMERA_MATRIX_RAW = [
    [336.1289796344718, 0.0, 308.15649184387826],
    [0.0, 302.3403978094867, 355.32041165989983],
    [0.0, 0.0, 1.0]
]

DISTANCE_COEFFICIENT = [
    [-0.3700034487259023],
    [ 0.2532078742688624],
    [ 0.031336655224801925],
    [ 0.008810303494888223],
    [-0.10860100898133669]
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

import math

###################### Camera Config Values ########################

MARKER_SIZE = 175
CAMERA_MATRIX_RAW = [
            [1856.5594482378056, 0.0, 971.2508020385866],
            [0.0, 1854.2218241989967, 472.4387993168578],
            [0.0, 0.0, 1.0]
        ]
DISTANCE_COEFFICIENT = [
            [0.4258768890441897],
            [-2.58573901335018],
            [-0.029226787102926456],
            [-0.005191425823853011],
            [ 6.748360238575704]
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

import math

def quaternion_to_yaw(q):
    """
    q: object with x, y, z, w 
    return: yaw in rad
    important: only works for yaw, object is in 2D plane
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw



def p_regulator(error, kp, max_output):
    """Proportional controller with saturation limits
    erroer: measurement error (setpoint minus actual value).
    kp: proportional gain factor.
    max_output: absolute maximum output."""

    output = kp * error

    if output > max_output:
        output = max_output
    elif output < -max_output:
        output = -max_output
    
    return output

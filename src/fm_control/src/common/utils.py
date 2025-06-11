import numpy as np
import math
from math import sqrt

MAX_ANGULAR_VEL = 2.84

def saturation(x, bound):
    """
    限制输入值在给定范围内
    """
    x = np.array(x)
    x_m = np.linalg.norm(x)
    if x_m > bound:
        return x / x_m * bound
    else:
        return x


def sign(x):
    x = np.array(x)
    y = []
    for number in x:
        if number > 0:
            y.append(1)
        elif number < 0:
            y.append(-1)
        else:
            y.append(0)
    return np.array(y)


def tanh_sign(x, k):
    x = np.array(x)
    return np.tanh(k * x)


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def u2vw_global(u, theta, offset):
    speed = sqrt(u[0] ** 2 + u[1] ** 2)
    # 如果速度太小 直接return
    if speed < 1e-3:
        return np.array([0.0, 0.0])

    # 投影到前向方向的线速度
    v = u[0] * math.cos(theta) + u[1] * math.sin(theta)
    # 朝向误差控制角速度
    omega = (u[0] * -math.sin(theta) + u[1] * math.cos(theta)) / offset

    omega = np.clip(omega, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)

    return np.array([v, omega])

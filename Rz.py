import numpy as np
import math


def Rz(a: float, x: float, y: float) -> list:
    rz = np.array([[math.cos(a), -math.sin(a), 0, 0], [math.sin(a), math.cos(a), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    t = np.dot(rz, np.array([x, y, 0, 0]))
    return t[0:2]

"""
x: 0 - 0.1337 --> phi: 0 - 3.141593
x: 0.06685, y: 0.11578 --> theta: pi/3 (1.0472)
"""

import math
from typing import Tuple

from scipy.optimize import fsolve
import numpy as np
from Rz import Rz


class IK:
    def __init__(self, l: float = 0.21, ite: int = 15) -> None:
        self.theta = None
        self.__phi_list = []
        self.phi = None
        self.L = l
        self.x = None
        self.y = None
        self.__init_guess = [0.0001, math.pi]
        self.__init_val = []
        self.__ite = ite

    def angles(self, x: float, y: float, a: float = 0.0) -> tuple[float, float]:
        x, y = Rz(a, x, y)
        if math.fabs(x) < 0.001:
            self.x = 0.0000001
        else:
            self.x = x

        if math.fabs(y) < 0.001:
            self.y = 0.0000001
        else:
            self.y = y

        self.theta = math.atan2(self.y, self.x)
        if math.fabs(self.theta) < 0.00001:
            self.theta = 0.0

        for i in range(0, self.__ite):
            guess = ((self.__init_guess[1] - self.__init_guess[0]) / 20) * i + self.__init_guess[0]
            self.__init_val.append(guess)

            # should use fulloutput=1 argument to get the error msgs to check the convergence
            val = fsolve(self.__equation, np.array([guess]))
            self.__phi_list.append(val[0])
        self.phi = self.__filtering(self.__phi_list)
        return self.theta, self.phi

    def __filtering(self, lis: list) -> float:
        lis = np.array(lis)
        mask = lis >= 0
        list_pos = lis[mask]
        mask = math.pi >= list_pos
        list_valid = list_pos[mask]
        uniq, count = np.unique(list_valid, return_counts=True)
        return uniq[np.argmax(count)]

    def __equation(self, a: np.ndarray) -> float:
        return math.sqrt(self.x ** 2 + self.y ** 2) - math.sqrt(self.L ** 2 * (1 - math.cos(a[0])) ** 2 / a[0] ** 2)


# ik = IK(ite=10)
# a, b = ik.angles(0.15, 0.0)
# print("Theta: ", ik.theta)
# print("Phi: ", b)

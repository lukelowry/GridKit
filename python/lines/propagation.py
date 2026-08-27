from collections import namedtuple

import numpy as np
from scipy.linalg import sqrtm

Propagation = namedtuple("Propagation", "gamma yc zc")


def propagation(Z, Y):
    gamma = np.array([sqrtm(product) for product in Z @ Y], dtype=Z.dtype)
    yc = np.linalg.solve(Z, gamma)
    return Propagation(gamma, yc, np.linalg.inv(yc))

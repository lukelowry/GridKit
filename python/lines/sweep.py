from collections import namedtuple

import numpy as np

from .modes import modes
from .propagation import propagation
from .reduction import incidence, reduce
from .series import SERIES, impedance
from .shunt import SHUNT, admittance

Sweep = namedtuple("Sweep", "omega R L G C gamma yc zc tv ti lam h tau")


def grid(fmin, fmax, points):
    return 2 * np.pi * np.geomspace(fmin, fmax, points)


def rlgc(Z, Y, omega):
    w = omega[:, None, None]
    return Z.real, Z.imag / w, Y.real, Y.imag / w


def sweep(line, omega, series=SERIES, shunt=SHUNT, reduced=True):
    Z, Y = impedance(omega, line, series), admittance(omega, line, shunt)
    if reduced:
        Z, Y = reduce(Z, Y, incidence(line))
    wave = propagation(Z, Y)
    return Sweep(omega, *rlgc(Z, Y, omega), *wave, *modes(omega, wave.gamma, line.length))

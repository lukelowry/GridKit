import numpy as np

from .geometry import logdistance
from .line import EPS0


def capacitive(omega, line):
    return 1j * omega[:, None, None] * 2 * np.pi * EPS0 * np.linalg.inv(logdistance(line))


def leakage(conductance):
    def term(omega, line):
        return np.eye(len(line.r)) * conductance * np.ones((len(omega), 1, 1))
    return term


SHUNT = (capacitive,)


def admittance(omega, line, terms=SHUNT):
    return sum(term(omega, line) for term in terms)

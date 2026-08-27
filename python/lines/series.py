import numpy as np
from scipy.special import ive, kve

from .geometry import logdistance, offsets
from .line import MU0


def geometric(omega, line):
    return 1j * omega[:, None, None] * MU0 / (2 * np.pi) * logdistance(line)


def internal(omega, line):
    m = np.sqrt(1j * omega[:, None] * line.mu * line.sigma)
    outer, inner, wall = m * line.r, m * line.q, m * (line.r - line.q)
    with np.errstate(invalid="ignore", divide="ignore"):
        core = np.where(line.q > 0, ive(1, inner) / kve(1, inner), 0) * np.exp(-wall - wall.real)
    z = m / (2 * np.pi * line.sigma * line.r) * (ive(0, outer) + kve(0, outer) * core) / (ive(1, outer) - kve(1, outer) * core)
    return np.eye(len(line.r)) * z[:, None, :]


def earth(omega, line):
    w = omega[:, None, None]
    dx, _, hs = offsets(line)
    p = 1 / np.sqrt(1j * w * MU0 * (line.earth.sigma + 1j * w * line.earth.eps))
    return 1j * w * MU0 / (4 * np.pi) * np.log1p(4 * p * (hs + p) / (dx**2 + hs**2))


SERIES = (geometric, internal, earth)


def impedance(omega, line, terms=SERIES):
    return sum(term(omega, line) for term in terms)

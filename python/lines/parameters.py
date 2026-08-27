from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np
from scipy.linalg import sqrtm
from scipy.special import ive, kve

from .line import EPS0, MU0


def logdistance(line):
    dx = line.x[:, None] - line.x
    direct = np.hypot(dx, line.h[:, None] - line.h)
    image = np.hypot(dx, line.h[:, None] + line.h)
    lam = np.log(image / np.where(np.eye(len(dx), dtype=bool), 1, direct))
    np.fill_diagonal(lam, np.log(2 * line.h / line.r))
    return lam


def internal_impedance(omega, line):
    m = np.sqrt(1j * omega[:, None] * line.mu * line.sigma)
    outer, inner, wall = m * line.r, m * line.q, m * (line.r - line.q)
    with np.errstate(invalid="ignore", divide="ignore"):
        core = np.where(line.q > 0, ive(1, inner) / kve(1, inner), 0) * np.exp(-wall - wall.real)
    return m / (2 * np.pi * line.sigma * line.r) * (ive(0, outer) + kve(0, outer) * core) / (ive(1, outer) - kve(1, outer) * core)


def earth_return(omega, line):
    w = omega[:, None, None]
    dx = line.x[:, None] - line.x
    hs = line.h[:, None] + line.h
    p = 1 / np.sqrt(1j * w * MU0 * (line.earth_sigma + 1j * w * line.earth_eps))
    return 1j * w * MU0 / (4 * np.pi) * np.log1p(4 * p * (hs + p) / (dx**2 + hs**2))


@dataclass(frozen=True)
class Effects:
    skin: Callable | None = internal_impedance
    earth: Callable | None = earth_return
    leakage: float | np.ndarray = 0.0


def zy(line, omega, effects=Effects()):
    w = omega[:, None, None]
    lam = logdistance(line)
    I = np.eye(len(lam))
    Z = 1j * w * MU0 / (2 * np.pi) * lam
    Y = 1j * w * 2 * np.pi * EPS0 * np.linalg.inv(lam) + I * effects.leakage
    if effects.skin:
        Z = Z + I * effects.skin(omega, line)[:, None, :]
    if effects.earth:
        Z = Z + effects.earth(omega, line)
    return Z, Y


def rlgc(line, omega, effects=Effects()):
    w = omega[:, None, None]
    Z, Y = zy(line, omega, effects)
    return Z.real, Z.imag / w, Y.real, Y.imag / w


def reduce(Z, Y, E):
    spread = np.linalg.solve(Z, np.broadcast_to(E, Z.shape[:1] + E.shape))
    return np.linalg.inv(E.T @ spread), E.T @ Y @ E


def response(Z, Y):
    gamma = np.array([sqrtm(product) for product in Z @ Y], dtype=Z.dtype)
    yc = np.linalg.solve(Z, gamma)
    return gamma, yc, np.linalg.inv(yc)

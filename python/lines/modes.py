from collections import namedtuple

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.sparse.csgraph import connected_components

Modes = namedtuple("Modes", "tv ti lam h tau")


def match(cost, maximize=False):
    return linear_sum_assignment(cost, maximize)[1]


def clusters(lam, gap):
    close = np.abs(lam[:, None] - lam) <= gap * (np.abs(lam)[:, None] + np.abs(lam))
    count, label = connected_components(close, directed=False)
    return [np.flatnonzero(label == c) for c in range(count)]


def canonical(lam, V):
    order = np.lexsort((lam.real, lam.imag))
    lam, V = lam[order], V[:, order]
    pivot = V[np.abs(V).argmax(axis=0), range(len(lam))]
    return lam, V * (pivot.conj() / np.abs(pivot))


def follow(lam, V, previous, gap):
    tv, ti, lam0 = previous
    raw = match(np.abs(ti.conj().T @ V), maximize=True)
    lam, V = lam[raw], V[:, raw]
    for c in clusters(lam, gap):
        Q, _ = np.linalg.qr(V[:, c])
        U, _, Wh = np.linalg.svd(Q.conj().T @ tv[:, c])
        V[:, c] = Q @ U @ Wh
        lam[c] = lam[c][match(np.abs(lam0[c][:, None] - lam[c]))]
    return lam, V


def modes(omega, gamma, length, gap=1e-8):
    spectra, bases = np.linalg.eig(gamma)
    tv, ti, lam = [], [], []
    for spectrum, basis in zip(spectra, bases):
        spectrum, basis = canonical(spectrum, basis) if not tv else follow(spectrum, basis, (tv[-1], ti[-1], lam[-1]), gap)
        tv.append(basis)
        ti.append(np.linalg.inv(basis).conj().T)
        lam.append(spectrum)
    tv, ti, lam = map(np.array, (tv, ti, lam))
    return Modes(tv, ti, lam, np.exp(-length * lam), length * lam.imag / omega[:, None])

import numpy as np


def incidence(line):
    keys = [None if p == "g" else (p, c if p in "abc" else 0) for p, c in zip(line.phase, line.circuit)]
    order = list(dict.fromkeys(k for k in keys if k))
    E = np.zeros((len(keys), len(order)))
    for i, key in enumerate(keys):
        if key:
            E[i, order.index(key)] = 1
    return E


def reduce(Z, Y, E):
    spread = np.linalg.solve(Z, np.broadcast_to(E, Z.shape[:1] + E.shape))
    return np.linalg.inv(E.T @ spread), E.T @ Y @ E

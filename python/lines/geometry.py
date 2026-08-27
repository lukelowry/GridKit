import numpy as np


def offsets(line):
    dx = line.x[:, None] - line.x
    return dx, line.h[:, None] - line.h, line.h[:, None] + line.h


def logdistance(line):
    dx, dh, hs = offsets(line)
    lam = np.log(np.hypot(dx, hs) / np.where(np.eye(len(dx), dtype=bool), 1, np.hypot(dx, dh)))
    np.fill_diagonal(lam, np.log(2 * line.h / line.r))
    return lam

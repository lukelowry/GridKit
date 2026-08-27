import argparse

import numpy as np

from .line import load, terminals
from .modes import modes
from .parameters import Effects, earth_return, internal_impedance, reduce, response, zy


def grid(fmin, fmax, points):
    return 2 * np.pi * np.geomspace(fmin, fmax, points)


def columns(name, values):
    if np.iscomplexobj(values):
        yield from columns(f"{name}_real", values.real)
        yield from columns(f"{name}_imag", values.imag)
        return
    for index, series in zip(np.ndindex(values.shape[1:]), values.reshape(len(values), -1).T):
        yield name + "".join(f"_{i}" for i in index), series


def write_response(path, omega, **fields):
    named = [("omega", omega)]
    named += [column for name, values in fields.items() for column in columns(f"Overhead_{name}", values)]
    np.savetxt(path, np.column_stack([v for _, v in named]), fmt="%.16e", delimiter=",", comments="", header=",".join(n for n, _ in named))


def sweep(line, omega, effects=Effects(), conductors=False):
    Z, Y = zy(line, omega, effects)
    if not conductors:
        Z, Y = reduce(Z, Y, terminals(line))
    gamma, yc, zc = response(Z, Y)
    m = modes(omega, gamma, line.length)
    w = omega[:, None, None]
    return dict(R=Z.real, L=Z.imag / w, G=Y.real, C=Y.imag / w, Yc=yc, Zc=zc,
                Tv=m.tv, Ti=m.ti, Alpha=m.lam.real, Beta=m.lam.imag, Tau=m.tau, H=m.h)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("line")
    parser.add_argument("-o", "--output", default="response.csv")
    parser.add_argument("--fmin", type=float, default=10.0)
    parser.add_argument("--fmax", type=float, default=1e8)
    parser.add_argument("--points", type=int, default=401)
    parser.add_argument("--conductors", action="store_true")
    parser.add_argument("--no-skin", action="store_true")
    parser.add_argument("--no-earth", action="store_true")
    parser.add_argument("--leakage", type=float, default=0.0)
    args = parser.parse_args()
    effects = Effects(skin=None if args.no_skin else internal_impedance,
                      earth=None if args.no_earth else earth_return,
                      leakage=args.leakage)
    omega = grid(args.fmin, args.fmax, args.points)
    write_response(args.output, omega, **sweep(load(args.line), omega, effects, args.conductors))


if __name__ == "__main__":
    main()

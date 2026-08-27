import argparse

import numpy as np

from .line import load
from .series import earth, geometric, internal
from .shunt import SHUNT, leakage
from .sweep import grid, sweep

def columns(name, values):
    if np.iscomplexobj(values):
        yield from columns(f"{name}_real", values.real)
        yield from columns(f"{name}_imag", values.imag)
        return
    for index, series in zip(np.ndindex(values.shape[1:]), values.reshape(len(values), -1).T):
        yield name + "".join(f"_{i}" for i in index), series


def write(path, result):
    fields = dict(R=result.R, L=result.L, G=result.G, C=result.C, Yc=result.yc, Zc=result.zc, Tv=result.tv, Ti=result.ti,
                  Alpha=result.lam.real, Beta=result.lam.imag, Tau=result.tau, H=result.h)
    named = [("omega", result.omega)] + [c for name, values in fields.items() for c in columns(f"Overhead_{name}", values)]
    np.savetxt(path, np.column_stack([v for _, v in named]), fmt="%.16e", delimiter=",", comments="", header=",".join(n for n, _ in named))


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
    series = (geometric,) + (() if args.no_skin else (internal,)) + (() if args.no_earth else (earth,))
    shunt = SHUNT + ((leakage(args.leakage),) if args.leakage else ())
    result = sweep(load(args.line), grid(args.fmin, args.fmax, args.points), series, shunt, not args.conductors)
    write(args.output, result)

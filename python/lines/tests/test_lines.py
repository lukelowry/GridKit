from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from lines import EPS0, MU0, Effects, Line, load, modes, reduce, response, rlgc, terminals, zy
from lines.parameters import internal_impedance

REPO = Path(__file__).resolve().parents[3]
LINES = REPO / "examples" / "EMT" / "Lines"
OVERHEAD = REPO / "examples" / "EMT" / "Overhead"

GOLDEN = {
    "69kv-wood-pole": ([-1.0668, 1.0668, -1.0668, 0.0], [14.9352, 13.7160, 12.4968, 10.3632]),
    "138kv-delta": ([-3.0, 3.0, 0.0, 0.0], [17.0, 17.0, 21.5, 25.0]),
    "345kv-horizontal": ([-7.4285, -6.9715, -0.2285, 0.2285, 6.9715, 7.4285, -4.3, 4.3],
                         [24.0, 24.0, 24.0, 24.0, 24.0, 24.0, 29.5, 29.5]),
    "500kv-double-circuit": ([-8.7285, -8.2715, -10.0285, -9.5715, -8.7285, -8.2715,
                              8.2715, 8.7285, 9.5715, 10.0285, 8.2715, 8.7285, -5.5, 5.5],
                             [30.0, 30.0, 38.0, 38.0, 46.0, 46.0, 30.0, 30.0, 38.0, 38.0, 46.0, 46.0, 53.0, 53.0]),
    "765kv-horizontal": ([-13.9285, -13.4715, -13.9285, -13.4715, -0.2285, 0.2285, -0.2285, 0.2285,
                          13.4715, 13.9285, 13.4715, 13.9285, -9.8, 9.8],
                         [31.7715, 31.7715, 32.2285, 32.2285, 31.7715, 31.7715, 32.2285, 32.2285,
                          31.7715, 31.7715, 32.2285, 32.2285, 40.0, 40.0]),
}

OMEGA = 2 * np.pi * np.geomspace(10, 1e8, 61)


def conductor(q=0.0, **fields):
    line = Line(x=np.array([0.0]), h=np.array([20.0]), r=np.array([0.01519]), q=np.array([q]),
                sigma=np.array([3.5e7]), mu=np.array([MU0]), phase=("a",), circuit=np.array([1]),
                length=1e5, earth_sigma=0.01, earth_eps=EPS0)
    return replace(line, **fields)


@pytest.mark.parametrize("name", GOLDEN)
def test_load_coordinates(name):
    path = LINES / f"{name}.line.json"
    line = load(path)
    x, H = GOLDEN[name]
    assert np.allclose(line.x, x)
    if "tension" in path.read_text():
        assert np.all(line.h < H)
    else:
        assert np.allclose(line.h, H)


def test_internal_impedance_limits():
    for q in (0.0, 0.00464):
        line = conductor(q)
        z = internal_impedance(np.array([1e-3]), line)[0, 0]
        assert np.isclose(z.real, 1 / (np.pi * line.sigma[0] * (line.r[0] ** 2 - q**2)), rtol=1e-6)
        if q == 0.0:
            assert np.isclose(z.imag / 1e-3, MU0 / (8 * np.pi), rtol=1e-4)
        omega = 2 * np.pi * 1e8
        depth = np.sqrt(2 / (omega * line.mu[0] * line.sigma[0]))
        z = internal_impedance(np.array([omega]), line)[0, 0]
        assert np.isclose(z.real * 2 * np.pi * line.r[0] * line.sigma[0] * depth, 1.0, rtol=2e-3)
        assert np.all(np.isfinite(internal_impedance(2 * np.pi * np.geomspace(1e-3, 1e8, 200), line)))


def test_parity_with_cpp_sweep():
    table = np.genfromtxt(OVERHEAD / "output" / "overhead.response.csv", delimiter=",", names=True)
    line = load(OVERHEAD / "overhead.line.json")
    R, L, G, C = rlgc(line, table["omega"])
    off = ~np.eye(3, dtype=bool)
    for name, values in (("R", R), ("L", L), ("C", C)):
        for i, j in zip(*np.nonzero(off if name != "C" else np.ones((3, 3), bool))):
            assert np.allclose(values[:, i, j], table[f"Overhead_{name}_{i}_{j}"], rtol=1e-6)
    assert np.all(G == 0)


def test_reduction():
    line = load(LINES / "345kv-horizontal.line.json")
    Z, Y = zy(line, OMEGA)
    E = terminals(line)
    Zt, Yt = reduce(Z, Y, E)
    K, P = E.shape
    system = np.block([[Z[0], -E], [E.T, np.zeros((P, P))]])
    rhs = np.vstack([np.zeros((K, P)), np.eye(P)])
    assert np.allclose(np.linalg.solve(system, rhs)[K:], Zt[0])
    assert np.allclose(Yt[0], E.T @ Y[0] @ E)
    assert Zt.shape == (len(OMEGA), 3, 3)

    shielded = load(LINES / "138kv-delta.line.json")
    Z, Y = zy(shielded, OMEGA)
    E = terminals(shielded)
    live, ground = E.any(axis=1), ~E.any(axis=1)
    kron = Z[:, live][:, :, live] - Z[:, live][:, :, ground] @ np.linalg.solve(Z[:, ground][:, :, ground], Z[:, ground][:, :, live])
    assert np.allclose(reduce(Z, Y, E)[0], kron)

    Z, Y = zy(conductor(), OMEGA)
    assert np.allclose(reduce(Z, Y, np.eye(1))[0], Z)


def test_ordering_and_effects():
    line = load(LINES / "345kv-horizontal.line.json")
    reverse = {f: getattr(line, f)[::-1] for f in ("x", "h", "r", "q", "sigma", "mu", "circuit")}
    reverse["phase"] = line.phase[::-1]
    R, L, G, C = rlgc(line, OMEGA)
    Rr, Lr, Gr, Cr = rlgc(replace(line, **reverse), OMEGA)
    for a, b in ((R, Rr), (L, Lr), (C, Cr)):
        assert np.allclose(a[:, ::-1, ::-1], b)
        assert np.allclose(a, np.swapaxes(a, 1, 2))
    R0, L0, G0, C0 = rlgc(line, OMEGA, Effects(earth=None))
    assert np.allclose(R0, np.eye(len(line.x)) * internal_impedance(OMEGA, line).real[:, None, :])
    R1, L1, G1, C1 = rlgc(line, OMEGA, Effects(skin=None, leakage=1e-11))
    assert np.allclose(np.diagonal(G1, axis1=1, axis2=2), 1e-11)
    assert np.allclose(np.diagonal(R1, axis1=1, axis2=2), np.diagonal(R - R0, axis1=1, axis2=2))


def test_response_and_modes():
    line = load(LINES / "345kv-horizontal.line.json")
    Z, Y = reduce(*zy(line, OMEGA), terminals(line))
    gamma, yc, zc = response(Z, Y)
    assert np.allclose(gamma @ gamma, Z @ Y)
    assert np.allclose(yc @ Z @ yc, Y)
    assert np.allclose(zc @ yc, np.eye(3))
    m = modes(OMEGA, gamma, line.length)
    eye = np.eye(3)
    assert np.allclose(np.conj(np.swapaxes(m.ti, 1, 2)) @ m.tv, eye, atol=1e-10)
    rebuilt = m.tv @ (m.lam[:, :, None] * np.linalg.inv(m.tv))
    assert np.allclose(rebuilt, gamma)
    assert np.all(np.isfinite(m.tau)) and np.all(m.tau > 0)
    assert np.allclose(m.h, np.exp(-line.length * m.lam))
    assert np.max(np.abs(np.diff(m.tv, axis=0))) < 0.5

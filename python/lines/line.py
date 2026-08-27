import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

MU0 = 4e-7 * np.pi
EPS0 = 8.8541878128e-12
EARTH_RADIUS = 6371008.8


@dataclass(frozen=True)
class Line:
    x: np.ndarray
    h: np.ndarray
    r: np.ndarray
    q: np.ndarray
    sigma: np.ndarray
    mu: np.ndarray
    phase: tuple
    circuit: np.ndarray
    length: float
    earth_sigma: float
    earth_eps: float


def load(path):
    path = Path(path)
    doc = json.loads(path.read_text())
    types, towers = catalogs(doc, path)
    entries = doc["conductors"]
    at = [towers[doc["tower"]]["attachments"][c["at"]] for c in entries]
    kind = [types[c["type"]] for c in entries]
    H = np.array([p["h"] for p in at])
    tension = np.array([c.get("tension", np.nan) for c in entries])
    weight = np.array([k["weight"] for k in kind])
    h, stretch = catenary(H, tension, weight, doc["path"]["span"])
    length = doc["path"].get("length") or route(doc["path"]["points"]) * stretch
    line = Line(
        x=np.array([p["x"] for p in at]),
        h=h,
        r=np.array([k["radius"]["outer"] for k in kind]),
        q=np.array([k["radius"].get("inner", 0.0) for k in kind]),
        sigma=np.array([k["conductivity"] for k in kind]),
        mu=np.array([k.get("permeability", 1.0) for k in kind]) * MU0,
        phase=tuple(c["phase"] for c in entries),
        circuit=np.array([c.get("circuit", 1) for c in entries]),
        length=length,
        earth_sigma=doc["earth"]["conductivity"],
        earth_eps=doc["earth"].get("permittivity", 1.0) * EPS0,
    )
    validate(line)
    return line


def catalogs(doc, path):
    sections = [doc.get("catalog", {})]
    sections += [json.loads((path.parent / f).read_text()) for f in doc.get("include", [])]
    merged = {"conductors": {}, "towers": {}}
    for section in sections:
        for kind, table in merged.items():
            for name, data in section.get(kind, {}).items():
                if name in table:
                    raise ValueError(f"duplicate {kind} type {name}")
                table[name] = data
    return merged["conductors"], merged["towers"]


def catenary(H, tension, weight, span):
    a = tension / weight
    eta = span / (2 * a)
    hung = ~np.isnan(tension)
    h = np.where(hung, H - a * (np.cosh(eta) - np.sinh(eta) / eta), H)
    return h, np.where(hung, np.sinh(eta) / eta, 1.0).mean()


def route(points):
    lat, lon = np.radians([[p["latitude"], p["longitude"]] for p in points]).T
    a = np.sin(np.diff(lat) / 2) ** 2 + np.cos(lat[:-1]) * np.cos(lat[1:]) * np.sin(np.diff(lon) / 2) ** 2
    return 2 * EARTH_RADIUS * np.arcsin(np.sqrt(a)).sum()


def validate(line):
    dx = line.x[:, None] - line.x
    dh = line.h[:, None] - line.h
    checks = [
        (np.all(line.r > line.q) and np.all(line.q >= 0), "radii"),
        (np.all(line.sigma > 0) and np.all(line.mu > 0), "conductor material"),
        (np.all(line.h > 0), "heights"),
        (np.all((dx**2 + dh**2 > 0) | np.eye(len(dx), dtype=bool)), "distinct positions"),
        (line.length > 0, "length"),
        (line.earth_sigma > 0 and line.earth_eps >= EPS0, "earth"),
        (set(line.phase) <= set("abcng") and set(line.phase) != {"g"}, "phases"),
    ]
    for ok, what in checks:
        if not ok:
            raise ValueError(f"invalid {what}")


def terminals(line):
    keys = [None if p == "g" else (p, c if p in "abc" else 0) for p, c in zip(line.phase, line.circuit)]
    order = list(dict.fromkeys(k for k in keys if k))
    E = np.zeros((len(keys), len(order)))
    for i, key in enumerate(keys):
        if key:
            E[i, order.index(key)] = 1
    return E

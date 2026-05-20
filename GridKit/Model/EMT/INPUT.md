# EMT Input Format

An EMT input file is a JSON object describing metadata, buses, components,
port wiring, and monitor output sinks.

```json
{
  "header": {
    "format_version": 1,
    "case_name": "Minimal EMT Scenario",
    "description": "Small EMT input example"
  },
  "buses": [
    {
      "name": "source_bus",
      "init": {
        "vm": 120.0,
        "va": 0.0
      }
    }
  ],
  "components": [],
  "monitors": [
    {
      "file_name": "case.csv",
      "format": "csv"
    }
  ]
}
```

## Root Object

| Field | Required | Type | Description |
| --- | --- | --- | --- |
| `header` | Yes | object | Scenario metadata. |
| `buses` | Yes | array | EMT bus objects. |
| `components` | Yes | array | EMT component objects. |
| `monitors` | No | array | Monitor sink objects. |

## Bus Object

| Field | Required | Type | Description |
| --- | --- | --- | --- |
| `name` | Yes | string | Bus identifier. |
| `init` | Yes | object | Bus initialization values. |
| `mon` | No | array | Ordered bus monitor variable names. |

Bus initialization uses RMS phase-voltage magnitude `vm` and phase-a angle
`va` in radians.

## Component Object

| Field | Required | Type | Description |
| --- | --- | --- | --- |
| `name` | Yes | string | Component identifier. |
| `class` | Yes | string | EMT component class. |
| `params` | Yes | object | Class-defined parameters. |
| `ports` | Yes | object | Class-defined port wiring. |
| `mon` | No | array | Ordered component monitor variable names. |

Active sparse-AD model classes are `BranchLumpedConstant`, `LoadRL`, and
`VoltageSource`. `Breaker` is currently parsed as a skeleton and does not add
solver equations.

## Ports

Electrical port values may be authored as bus names or integer bus IDs.

```text
BranchLumpedConstant: from, to
LoadRL:               ac
VoltageSource:        bus
Breaker:              from, to
```

## Required Parameters

`VoltageSource`:

- `e`: three RMS phase voltages, each nonnegative.
- `phi`: three phase angles in radians.
- `r`: three phase resistances, each strictly positive.
- `frequency` in Hz or `omega0` in rad/s. If both are present, they must agree.

`LoadRL`:

- `r`: three phase resistances, each nonnegative.
- `l`: three phase inductances, each strictly positive.

`BranchLumpedConstant`:

- `r`: finite 3x3 resistance matrix in ohm/m.
- `l`: finite 3x3 inductance matrix in H/m; the derived matrix must be nonsingular.
- `g`: optional finite 3x3 conductance matrix in S/m; defaults to zero.
- `c`: finite symmetric 3x3 capacitance matrix in F/m with strictly positive diagonal entries.
- `length`: finite positive length in m.

Branch shunt capacitance is mandatory. A zero-capacitance branch is a validation
error, not a fallback mode.

## Monitor Sinks

`monitors` declares output sinks. If `file_name` is omitted, the sink writes to
standard output. `format` accepts `csv`, `json`, or `yaml`; `delim` defaults to
`,` for CSV.

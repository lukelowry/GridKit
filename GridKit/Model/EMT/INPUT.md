# EMT Input Format

An EMT input file is a JSON object describing metadata, buses, components,
port wiring, and monitor output sinks.

```json
{
  "header": {
    "format_version": 1,
    "case_name": "Minimal EMT Case",
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
| `header` | Yes | object | Case metadata. |
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

Supported classes in this scaffold are `BranchLumpedConstant`, `LoadRL`,
`VoltageSource`, and `Breaker`.

## Ports

Electrical port values may be authored as bus names or integer bus IDs.

```text
BranchLumpedConstant: from, to
LoadRL:               ac
VoltageSource:        bus
Breaker:              from, to
```

## Monitor Sinks

`monitors` declares output sinks. If `file_name` is omitted, the sink writes to
standard output. `format` accepts `csv`, `json`, or `yaml`; `delim` defaults to
`,` for CSV.

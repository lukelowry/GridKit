# EMT Input Format

An EMT input file is a JSON object that describes one electromagnetic
transients case: metadata, buses, components, port wiring, and monitor output
sinks.

This document defines the file-level schema. Component-specific `params`,
`ports`, and `mon` entries are defined by the component model documentation.

## General Rules

- Required root arrays must be present. Use `[]` when a category is empty.
- `class` values are case-sensitive and must match an EMT component type.
- Unknown fields are invalid.
- Object names must be identifiers and must be unique across buses and
  components.
- Ports are the only authored component-boundary wiring.

```text
EMT case file
+-- header
+-- buses[]
|   +-- init
+-- components[]
|   +-- params
|   +-- ports
+-- monitors[] optional
```

## Case Object

The root JSON object has these fields:

| Field | Required | Type | Description |
| --- | --- | --- | --- |
| `header` | Yes | object | Case metadata. |
| `buses` | Yes | array | Bus objects. |
| `components` | Yes | array | Component objects. |
| `monitors` | No | array | Monitor sink objects. |

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

## Header Object

| Field | Required | Type | Description |
| --- | --- | --- | --- |
| `format_version` | Yes | integer | Input format version. The supported value is `1`. |
| `case_name` | No | string | Case name. |
| `description` | No | string | Case description. |

## Component Objects

The model topology is split across two arrays:

- `buses` contains EMT bus objects.
- `components` contains EMT component objects.

### Bus Object

| Field | Required | Type | Description |
| --- | --- | --- | --- |
| `name` | Yes | string | Bus identifier. |
| `init` | Yes | object | Bus initialization values. |
| `mon` | No | array | Ordered bus monitor variable names. |

Bus initialization is a nested object:

| Field | Required | Type | Description |
| --- | --- | --- | --- |
| `vm` | Yes | number | Initial RMS phase voltage magnitude. |
| `va` | Yes | number | Initial phase-a voltage angle in radians. |

```json
{
  "name": "receiving_bus",
  "init": {
    "vm": 120.0,
    "va": 0.0
  },
  "mon": ["va", "vb", "vc"]
}
```

### Component Object

| Field | Required | Type | Description |
| --- | --- | --- | --- |
| `name` | Yes | string | Component identifier. |
| `class` | Yes | string | EMT component class. |
| `params` | Yes | object | Class-defined component parameters. |
| `ports` | Yes | object | Class-defined port wiring. |
| `mon` | No | array | Ordered component monitor variable names. |

```json
{
  "name": "line",
  "class": "BranchLumpedConstant",
  "params": {
    "...": "..."
  },
  "ports": {
    "from": "source_bus",
    "to": "receiving_bus"
  },
  "mon": ["ia", "ib", "ic"]
}
```

## Ports

`ports` is a flat object whose keys are class-defined port names.

```text
Electrical port value -> bus name
Input port value      -> producer_component.output_port
Output port           -> declared by producer class, not authored on producer
```

Every required electrical and input port must appear exactly once. Unknown
ports are invalid. Output ports are provided by their owning component class and
are named only when another component connects to them.

Electrical port example:

```json
{
  "name": "source",
  "class": "VoltageSource",
  "params": {
    "...": "..."
  },
  "ports": {
    "bus": "source_bus"
  }
}
```

Input port example for custom EMT components:

```json
{
  "name": "consumer",
  "class": "MockPortConsumer",
  "params": {},
  "ports": {
    "in": "producer.out"
  }
}
```

## Monitor Sinks

Buses and components select monitor variables with `mon`. The listed order is
the requested output order.

`monitors` declares output sinks. If `monitors` is omitted, no monitor output is
started. If `file_name` is omitted, the sink writes to standard output.

| Field | Required | Type | Description |
| --- | --- | --- | --- |
| `file_name` | No | string | Output file path. If omitted, write to standard output. |
| `format` | Yes | string | Output format: `csv`, `json`, or `yaml`. |
| `delim` | No | string | CSV delimiter. Default is `","`. |

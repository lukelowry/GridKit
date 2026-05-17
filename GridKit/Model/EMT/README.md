# Electromagnetic Transients (EMT)

This directory contains design documentation for electromagnetic transient
(EMT) component models in instantaneous abc coordinates.

## Conventions

These conventions reflect the current EMT model draft and may change as the
EMT design and implementation develop.

- Phase order is $a$, $b$, $c$.
- Equations use SI units unless a model says otherwise.
- Current injection terms are written as positive into buses.


## Model Categories

The current EMT documentation is organized into two categories:
- `Bus`
- `Component`

Branch models such as `BranchLumpedConstant` and `BranchFrequencyDependent` are documented under
`Component/Branch` because they are EMT components connected to buses.

## System Model

`Case.hpp` is the public case-loading façade: it defines `CaseData`,
`CaseNames`, `Case`, and `loadCase`. The JSON schema implementation lives in
`IO/CaseJson.hpp`, with scalar/vector/matrix readers in `IO/JsonSupport.hpp`
and descriptor parameter binding in `IO/ParamReader.hpp`.
The EMT case input format is documented in [INPUT.md](INPUT.md).
Dense rational approximation data is stored in sidecar `.fit.json` files
documented in [IO/FIT_JSON.md](IO/FIT_JSON.md). Case JSON references those
files from component parameters and keeps the case topology readable.

The EMT `SystemModel` is the IDA-facing runtime model. `SystemModelData` owns
construction-time buses, typed component storage, port connections, and
signal connections. EMT requires Enzyme; without Enzyme the EMT target is not
configured. `Layout` assigns global `y` variables and residual rows once:

```text
global y / f
+--------------+------------------+
| Bus voltages | Component vars   |
+--------------+------------------+
```

Models read through `StateView` and write equations and KCL injections through
`EquationView`. Components expose only initialization and residual physics;
`JacobianPlan` owns cached Enzyme workspaces and CSR insertion slots.
Electrical wiring is represented by
`PortConnection` entries from component ports to buses. Signal/control
wiring is represented by direction-specific `OutputPortRef` to `InputPortRef`
`SignalPortConnection` entries.

## Monitoring

EMT case files declare monitored variables inline on the entity that owns them.
Buses and components accept an optional `mon` array, for example
`"mon": ["va", "vb"]` on a bus or `"mon": ["ia", "ib"]` on a component.
The output label is always the entity `name`, so CSV headers are emitted as
`<name>_<variable>`.

The top-level `monitors` key is only the sink list:

```json
"monitors": [
  { "file_name": "case.csv", "format": "csv", "delim": "," }
]
```

The former EMT case shape with `monitors.sinks`, `monitors.buses`, and
`monitors.components` is intentionally unsupported.

## Open Design Notes

Distributed propagation and delay/history terms are placeholders until internal
signal delay support is designed. `BranchFrequencyDependent` currently consumes
only characteristic-admittance `Yc` rational fits.
- Initial electrical wiring will use Delta configuration only

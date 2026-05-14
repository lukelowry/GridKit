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

Branch models such as `BranchLumpedConstant` are documented under
`Component/Branch` because they are EMT components connected to buses.

## System Model

The EMT `SystemModel` is the IDA-facing runtime model. `NetworkData` owns
construction-time buses, typed component storage, terminal connections, and
port connections. `Layout` assigns global `y` variables and residual rows once:

```text
global y / f
+--------------+------------------+
| Bus voltages | Component vars   |
+--------------+------------------+
```

Models read through `VariableView`, write equations and KCL injections through
`ResidualView`, declare exact sparse structure through `PatternView`, and write
Jacobian values through `JacobianView`. Electrical wiring is represented by
`TerminalConnection` entries from component terminals to buses. Signal/control
wiring is represented by direction-specific `OutputRef` to `InputRef`
`PortConnection` entries.

## Open Design Notes

Distributed parameter lines are placeholders until internal signal delay support
is designed.
- Initial electrical wiring will use Delta configuration only

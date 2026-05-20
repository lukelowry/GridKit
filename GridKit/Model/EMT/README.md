# Electromagnetic Transients (EMT)

This directory contains the simplified EMT architecture scaffold. It follows
the same lifecycle and build conventions as `PhasorDynamics`, while keeping
the EMT model set in instantaneous abc coordinates.

## Conventions

- Phase order is `a`, `b`, `c`.
- Equations use SI units unless a model says otherwise.
- Current injection terms are written as positive into buses.
- This branch intentionally contains architecture stubs only; numerical EMT
  equations are documented in component READMEs and will be implemented later.

## Architecture

The EMT skeleton uses:

- `GridElement` as the common `Model::Evaluator` base.
- `Component` as the base for branch/load/source/breaker models.
- One concrete `Bus` implementation.
- `SystemModel` with separate bus and component containers so buses are
  allocated, initialized, and residual-evaluated first.

This branch does not include the older EMT `Case`, `IO`, `System`, `Layout`,
`Views`, `ComponentStore`, event scheduler, or Jacobian runtime architecture.

## Model Categories

- `Bus`
- `Components/BranchLumpedConstant`
- `Components/LoadRL`
- `Components/VoltageSource`
- `Components/Breaker`

## History Buffer Reference

Future distributed-line work is expected to need history buffering and
polynomial/linear interpolation. The current reference point in
`lukel/emt-system-dev` is
`GridKit/Model/EMT/Math/RationalApprox/README.md`, which notes that travel-time
delay and history interpolation are separate line-model concerns. No
implemented history-buffer model is ported in this branch.

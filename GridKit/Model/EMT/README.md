# Electromagnetic Transients (EMT)

The EMT implementation is a sparse-AD-only `SystemModel` for instantaneous
abc models. Components are value-like equation objects: each target model has
one templated residual over local `y`, `yp`, and `f` arrays, and `SystemModel`
owns all solver-facing global vectors and CSR Jacobian storage.

## Local Layout

For every component, local arrays are ordered as:

```text
y / yp: [ own variables | port 0 node variables | port 1 node variables | ... ]
f:      [ own equations | port 0 node injections | port 1 node injections | ... ]
```

Own residual rows are assigned into the global residual. Port rows are current
injections accumulated into bus KCL equations. Electrical ports are
three-phase and use phase order `a,b,c`.

## Sparse Jacobian

The sparse pattern is discovered once with dependency tracking on the same
single component residual used for values. Local `y` and `yp` are tracked as
distinct variable ranges, and `SystemModel::tag()` is derived from structural
`dF/dyp` columns. CSR coordinates are deduplicated and local contributions are
bound to stable slots before time stepping.

Every Jacobian evaluation clears CSR values before adding component
contributions. The nonlinear sparse-AD value path is centralized in
`System/SparseAD.hpp`; the current production EMT models are affine and use
cached AD coefficients from the pattern pass.

## Models

- `Bus`: fixed three-phase node block initialized from RMS voltage phasors.
- `Components/VoltageSource`: Norton source injection through phase resistance.
- `Components/LoadRL`: three-phase differential RL load current.
- `Components/BranchLumpedConstant`: full 3x3 coupled nominal pi branch.
- `Components/Breaker`: parsed skeleton only unless separately implemented.

`BranchLumpedConstant` requires a finite, symmetric shunt capacitance matrix
with strictly positive diagonal entries. There is no zero-capacitance branch
mode.

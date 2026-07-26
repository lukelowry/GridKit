# Electromagnetic Transients (EMT)

This directory documents electromagnetic transient (EMT) model specifications
and reusable operators in instantaneous phase coordinates.

> [!NOTE]
> The formulation supports $N$ phases; initial development targets three
> phases.

## Conventions

- $N$ denotes a phase or output dimension, $K$ a conductor or input dimension,
  $M$ a mode count, and $Q$ a pole count.
- Corresponding calligraphic symbols denote index sets, such as
  $\mathcal{M}$ for modal indices and $\mathcal{Q}$ for pole indices.
- Equations use SI units unless a model states otherwise.
- Time derivatives are written explicitly as $\mathrm{d}(\cdot)/\mathrm{d}t$.
- Current injection terms are written as positive into buses.
- $\mathrm{j}$ denotes the imaginary unit and is not used as an index.
- The Model Variables section lists DAE variables owned directly by the model.
  Each scalar internal variable corresponds to one local residual row;
  external variables are owned elsewhere. Submodel-owned variables are not
  repeated by the parent.
- Wiring introduces aliases or derived signals, not DAE variables or residual
  rows. Ports and monitors may expose either without changing ownership.
- Equation headings follow the assembled DAE row classification; connected
  models may add derivative terms to a residual that contains none locally.
- Case data lists model parameters under `params` and each submodel instance
  under `submodels` by its JSON name. A submodel block carries that submodel's
  own parameters, so a rational submodel block is a
  [VectorFit](Operators/Rational/VectorFit/README.md) parameter set.

## Initialization

RMS/phasor conversion, when used, is an external responsibility. EMT
initialization receives only real-valued, instantaneous time-domain data.

```math
\mathbf{F}
\left(t_0,\mathbf{y}_\mathrm{d},\mathbf{y}_\mathrm{a},
\dot{\mathbf{y}}_\mathrm{d};\mathbf{q}\right)=\mathbf{0},
```

where $\mathbf{q}$ contains discrete inputs. The initial-state provider supplies
the differential states $\mathbf{y}_\mathrm{d}$ and any required delay history.
The consistent DAE solve preserves those states and determines the algebraic
variables $\mathbf{y}_\mathrm{a}$ and differential-state derivatives
$\dot{\mathbf{y}}_\mathrm{d}$.

After a discrete event, all commands at the event time are applied atomically.
The same consistent solve preserves the differential states across the event
and recomputes the algebraic variables and differential-state derivatives. It
fails if the new configuration has no finite consistent solution.

### Initial-State File

A JSON document paired with the case supplies the initial state.

Key | Description
--- | -----------
`format_version`, `format_revision` | File format identifiers
`case_name` | Matches the system model
`time` | Initial time $t_0$ [sec]
`buses` | Entries of `id` and `variables`
`components` | Entries of `id`, `variables`, and `submodels`

Each variable value matches the dimension listed for it under Model Variables,
and submodel states nest under `submodels` by submodel instance name. Every
differential variable in the assembled system is supplied exactly once;
supplying an algebraic variable is an error.

Each model documents its own variables in an Internal Initialization table.
Models whose initialization data are not DAE variables, such as delay
prehistory, state the requirement in prose instead. The `Source` column takes
one of the following values.

Source | Meaning
------ | -------
Initial state | Read from the initial-state file under the listed JSON name
Consistent solve | Determined by the consistent DAE solve
Connected bus | Supplied by the connected EMT bus
Signal source | Supplied by the connected signal before the consistent solve

## Contents

- [Bus](Bus/README.md)
- [Components](Component/README.md)
- [Operators](Operators/README.md)

## References

- B. Gustavsen and A. Semlyen, [“Rational approximation of frequency domain
  responses by vector fitting,”](https://doi.org/10.1109/61.772353) *IEEE
  Transactions on Power Delivery*, 1999.
- A. Morched, B. Gustavsen, and M. Tartibi, [“A universal model for accurate
  calculation of electromagnetic transients on overhead lines and underground
  cables,”](https://doi.org/10.1109/61.772350) *IEEE Transactions on Power
  Delivery*, 1999.
- B. Gustavsen and A. Semlyen, [“Enforcing passivity for admittance matrices
  approximated by rational functions,”](https://doi.org/10.1109/59.910786)
  *IEEE Transactions on Power Systems*, 2001.

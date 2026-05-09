# EMT

This directory contains electromagnetic transient models. EMT models use
instantaneous phase-domain variables and are organized with the same
documentation discipline as `PhasorDynamics`, while keeping ownership and
assembly concerns separated more cleanly.

## Architecture

- `Component` is the EMT-local base for buses, lines, and future network
  devices.
- `SystemModel` is the solver-facing evaluator. It owns buses and components
  with `std::unique_ptr`, owns the connection table, freezes topology at
  allocation, and assembles residuals and Jacobians additively.
- `RationalApprox` is a reusable non-component equation block for vector-fitted
  matrix rational approximations.
- `Bus` owns abc voltage variables and current-balance residual rows.
- `Line` owns terminal characteristic-admittance memory states and exposes
  terminal currents. It does not own or store bus pointers.
- `VoltageSource` constrains a connected bus to balanced abc voltage and
  exposes the source current needed by current balance.
- `ShuntLoad` contributes abc shunt conductance current to a connected bus.

Topology is declared separately from component parameters:

```cpp
auto& bus1 = system.addBus(bus1_data);
auto& bus2 = system.addBus(bus2_data);
auto& line = system.addLine(line_data);

system.connect(line.terminal(0), bus1);
system.connect(line.terminal(1), bus2);
```

At allocation, `SystemModel` binds each terminal to the connected bus voltage
view and current-balance row. During residual and Jacobian evaluation, terminal
contributions are assembled through this connection table.

Bus residuals use a simple current-balance convention. For bus $b$, let
$\mathcal{C}(b)$ be the connected terminals and devices, and let
$\mathbf{i}_{\eta \rightarrow b}(t)$ be the current contribution entering the
bus from connection $\eta$. Then

```math
\mathbf{0}
= \mathbf{r}_b(t)
= \sum_{\eta \in \mathcal{C}(b)} \mathbf{i}_{\eta \rightarrow b}(t)
```

## Model Documentation

Each EMT model folder must provide a local `README.md` following the same
documentation architecture and notation used by `PhasorDynamics` models:

```text
# <Model Name> Model

## Model Parameters

## Model Variables
### Internal Variables
#### Differential
#### Algebraic
### External Variables
#### Differential
#### Algebraic

## Model Equations
### Differential Equations
### Algebraic Equations

## Initialization

## Model Outputs
```

Use the same table columns as the corresponding `PhasorDynamics` documents:
`Symbol`, `Units`, `Description`, and `Note` when documenting variables and
parameters. Use bold capital symbols for matrices, lowercase bold symbols for
vectors, and explicit residual sign conventions for any quantity assembled into
a system equation.

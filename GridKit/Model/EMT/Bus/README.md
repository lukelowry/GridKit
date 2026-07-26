# Bus Model

`Bus` represents an $N$-phase bus in instantaneous phase coordinates. It owns
the differential bus voltage and contributes the current-balance residual to
the assembled DAE. $\mathcal{E}$ denotes the set of connected devices.

## Block Diagram

![Bus model block diagram](../../../../docs/Figures/EMT/Bus/diagram.png)

Figure 1: Bus model

## Model Parameters

Symbol | Units | JSON | Description | Note
------ | ----- | ---- | ----------- | ----
$N$ | [-] | `N` | Number of phases | Required, positive integer

### Parameter Validation

```math
N \in \mathbb{Z}_{>0}
```

### Derived Parameters

None.

## Submodels

None.

### Submodel Validation

None.

## Model Variables

### Internal Variables

#### Differential

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{v}$ | [V] | Bus voltage vector | $\mathbf{v} \in \mathbb{R}^N$

#### Algebraic

None.

### External Variables

#### Differential

None.

#### Algebraic

None.

## Model Ports

Symbol | Port | Type | Units | Description | Note
------ | ---- | ---- | ----- | ----------- | ----
$\mathbf{i}_e$ | `i` | Input | [A] | Current from connected device $e$ | One port per $e \in \mathcal{E}$, $\mathbf{i}_e \in \mathbb{R}^N$
$\mathbf{v}$ | `v` | Output | [V] | Bus voltage supplied to connected devices | $\mathbf{v} \in \mathbb{R}^N$

## Model Equations

### Differential Equations

```math
0 = \sum_{e \in \mathcal{E}} \mathbf{i}_e
```

### Algebraic Equations

None.

### Wiring

None.

## Initialization

### Input Initialization

None. Connected components contribute their residual equations to the
assembled system; they do not initialize the bus.

### Internal Initialization

Symbol | JSON | Source | Note
------ | ---- | ------ | ----
$\mathbf{v}$ | `v` | Initial state | Differential bus
$\mathbf{v}$ | — | Consistent solve | Algebraic bus, `v` is not supplied

The solve determines $\mathrm{d}\mathbf{v}/\mathrm{d}t$, and current balance is
an initialization invariant:

```math
\sum_{e \in \mathcal{E}}\mathbf{i}_e=\mathbf{0}.
```

For an algebraic bus connected only through dynamic current branches, the
runtime model uses differentiated KCL to keep the assembled DAE index one.

### Output Initialization

None.

## Monitors

Monitor | Units | Description | Note
------- | ----- | ----------- | ----
`v` | [V] | Bus voltage | $\mathbf{v} \in \mathbb{R}^N$

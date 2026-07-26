# Switch Model

`Switch` represents an ideal three-phase EMT switch between two buses. Series
current $\mathbf{i}_{12}$ is directed from terminal 1 to terminal 2. The required
Boolean input `open` operates all three phases: `true` is open and `false` is
closed. The switch contains no energy storage; switching transients arise from
the connected EMT network.

## Block Diagram

![Switch model block diagram](../../../../../docs/Figures/EMT/Switch/diagram.png)

Figure 1: Switch model

## Model Parameters

None.

### Parameter Validation

None.

### Derived Parameters

None.

## Submodels

None.

### Submodel Validation

None.

## Model Variables

### Internal Variables

#### Differential

None.

#### Algebraic

Symbol | Units | JSON | Description | Note
------ | ----- | ---- | ----------- | ----
$\mathbf{i}_{12}$ | [A] | — | Series current from terminal 1 to terminal 2 | $\mathbf{i}_{12} \in \mathbb{R}^3$

### External Variables

#### Differential

Symbol | Units | JSON | Description | Note
------ | ----- | ---- | ----------- | ----
$\mathbf{v}_1$ | [V] | — | Terminal 1 voltage owned by EMT bus | $\mathbf{v}_1 \in \mathbb{R}^3$
$\mathbf{v}_2$ | [V] | — | Terminal 2 voltage owned by EMT bus | $\mathbf{v}_2 \in \mathbb{R}^3$

#### Algebraic

None.

## Model Ports

Symbol | Port | Type | Units | Description | Note
------ | ---- | ---- | ----- | ----------- | ----
$\mathbf{v}_1$ | `v1` | Input | [V] | Terminal 1 bus voltage | $\mathbf{v}_1 \in \mathbb{R}^3$
$\mathbf{v}_2$ | `v2` | Input | [V] | Terminal 2 bus voltage | $\mathbf{v}_2 \in \mathbb{R}^3$
$\mathrm{open}$ | `open` | Input | [-] | Ganged three-phase switch command | Required Boolean; `true` open, `false` closed
$\mathbf{i}_1$ | `i1` | Output | [A] | Current injection at terminal 1 | $\mathbf{i}_1 \in \mathbb{R}^3$
$\mathbf{i}_2$ | `i2` | Output | [A] | Current injection at terminal 2 | $\mathbf{i}_2 \in \mathbb{R}^3$

## Model Equations

### Differential Equations

None.

### Algebraic Equations

```math
\begin{cases}
\mathbf{i}_{12} = \mathbf{0}, & \text{open}, \\
\mathbf{v}_2-\mathbf{v}_1 = \mathbf{0}, & \text{closed}.
\end{cases}
```

The series current is algebraic in both positions, so neither terminal bus may
require a differentiated current balance. A terminal bus therefore needs a
shunt conductance or capacitance of its own, which a purely resistive branch
supplies.

The model reserves the union Jacobian entries with respect to
$\mathbf{i}_{12}$, $\mathbf{v}_1$, and $\mathbf{v}_2$ in both positions, so
switching changes residual and Jacobian values without changing dimensions or
sparsity. After `open` changes, the solver preserves differential states and
recomputes consistent algebraic variables and differential-state derivatives.
The connected network must admit a consistent finite solution in the commanded
position.

### Wiring

```math
\begin{aligned}
\mathbf{i}_1 &\leftarrow -\mathbf{i}_{12} \\
\mathbf{i}_2 &\leftarrow \mathbf{i}_{12}.
\end{aligned}
```

## Initialization

None beyond the EMT initialization contract. The series current is algebraic
and the consistent solve determines it subject to the commanded position.

Closing fails if the preserved differential states of the connected network
are incompatible with $\mathbf{v}_2=\mathbf{v}_1$. Opening fails if the
network cannot carry $\mathbf{i}_{12}=\mathbf{0}$ with finite terminal
voltages, which is the case when the switched branch is the only path for a
series inductance.

## Monitors

Monitor | Units | Description | Note
------- | ----- | ----------- | ----
`open` | [-] | Switch command | Boolean; `true` open, `false` closed
`i12` | [A] | Series current from terminal 1 to terminal 2 | $\mathbf{i}_{12} \in \mathbb{R}^3$

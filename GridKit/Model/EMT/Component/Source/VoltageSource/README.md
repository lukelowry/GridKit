# VoltageSource Model

`VoltageSource` represents an $N$-phase sinusoidal EMT voltage source connected
to the EMT bus through a series terminal impedance. Current $\mathbf{i}$ is
injected from the source into the EMT bus.

## Block Diagram

![VoltageSource model block diagram](../../../../../../docs/Figures/EMT/VoltageSource/diagram.png)

Figure 1: VoltageSource model

## Model Parameters

Symbol | Units | JSON | Description | Note
------ | ----- | ---- | ----------- | ----
$N$ | [-] | `N` | Number of phases | Required, positive integer
$\mathbf{E}$ | [V] | `E` | Source voltage magnitudes | $\mathbf{E} \in \mathbb{R}_{\ge 0}^N$, RMS
$\boldsymbol{\phi}$ | [rad] | `phi` | Source phase offsets | $\boldsymbol{\phi} \in \mathbb{R}^N$
$\omega$ | [rad/s] | `omega` | Source angular frequency | Required, positive

### Parameter Validation

```math
\begin{aligned}
N &\in \mathbb{Z}_{>0} \\
\mathbf{E} &\in \mathbb{R}_{\ge 0}^N \\
\boldsymbol{\phi} &\in \mathbb{R}^N \\
\omega &> 0
\end{aligned}
```

### Derived Parameters

Define the phase-index set

```math
\mathcal{N} = \{1,\ldots,N\}.
```

## Submodels

Symbol | Description | Type | Order | JSON | Inputs | Outputs
------ | ----------- | ---- | ----- | ---- | ------ | -------
$\mathbf{z}$ | Series terminal impedance | [VectorFit](../../../Operators/Rational/VectorFit/README.md) | $NQ_{\mathbf{z}}$ | `Z` | $\mathbb{R}^N$ | $\mathbb{R}^N$

### Submodel Validation

The current is differential for a nonsingular linear coefficient and algebraic
when the coefficient is zero. Partially singular coefficients are not
supported.

```math
\mathbf{E}^{\mathbf{z}}=\mathbf{0}
\qquad \text{or} \qquad
\mathrm{rank}(\mathbf{E}^{\mathbf{z}})=N.
```

## Model Variables

### Internal Variables

#### Differential

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{i}$ | [A] | Current injection from source into EMT bus | $\mathbf{i} \in \mathbb{R}^N$, $\mathrm{rank}(\mathbf{E}^{\mathbf{z}})=N$

#### Algebraic

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{e}$ | [V] | Source voltage vector | $\mathbf{e} \in \mathbb{R}^N$
$\mathbf{i}$ | [A] | Current injection from source into EMT bus | $\mathbf{i} \in \mathbb{R}^N$, $\mathbf{E}^{\mathbf{z}}=\mathbf{0}$

### External Variables

#### Differential

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{v}$ | [V] | Bus voltage vector owned by EMT bus | $\mathbf{v} \in \mathbb{R}^N$

#### Algebraic

None.

## Model Ports

Symbol | Port | Type | Units | Description | Note
------ | ---- | ---- | ----- | ----------- | ----
$\mathbf{v}$ | `v` | Input | [V] | Bus voltage at source port | $\mathbf{v} \in \mathbb{R}^N$
$\mathbf{i}$ | `i` | Output | [A] | Current injection at source port | $\mathbf{i} \in \mathbb{R}^N$

## Model Equations

### Differential Equations

For $\mathrm{rank}(\mathbf{E}^{\mathbf{z}})=N$,

```math
0 = \mathbf{z}[\mathbf{i}] + \mathbf{v} - \mathbf{e}
```

### Algebraic Equations

```math
0 = e_n - \sqrt{2}E_n\cos(\omega t + \phi_n),
\quad n \in \mathcal{N}
```

For $\mathbf{E}^{\mathbf{z}}=\mathbf{0}$, the current residual is algebraic.

### Wiring

None.

## Initialization

### Input Initialization

Symbol | JSON | Source | Note
------ | ---- | ------ | ----
$\mathbf{v}$ | — | Connected bus | Terminal voltage at $t_0$

### Internal Initialization

Symbol | JSON | Source | Note
------ | ---- | ------ | ----
$\mathbf{i}$ | `i` | Initial state | $\mathrm{rank}(\mathbf{E}^{\mathbf{z}})=N$, solve determines $\mathrm{d}\mathbf{i}/\mathrm{d}t$
$\mathbf{i}$ | — | Consistent solve | $\mathbf{E}^{\mathbf{z}}=\mathbf{0}$
$\mathbf{e}$ | — | Consistent solve | $e_n(t_0)=\sqrt{2}E_n\cos(\omega t_0+\phi_n)$
$\mathbf{w}_q$, $\mathbf{v}_q$ | `Z` | Initial state | Impedance memory states

### Output Initialization

None.

## Monitors

Monitor | Units | Description | Note
------- | ----- | ----------- | ----
`e` | [V] | Source voltage | $\mathbf{e} \in \mathbb{R}^N$
`i` | [A] | Source current injection | $\mathbf{i} \in \mathbb{R}^N$

## Development

The initial three-phase formulation fixes $N=3$ and requires
$Q_{\mathbf{z}}=0$, so the series terminal impedance reduces to a resistance
and inductance and $\mathbf{i}$ is a differential variable.

### Derived Parameters

```math
\mathbf{R}_\mathrm{s} = \mathbf{D}^{\mathbf{z}},
\qquad
\mathbf{L}_\mathrm{s} = \mathbf{E}^{\mathbf{z}}
```

### Differential Equations

```math
0 =
\mathbf{R}_\mathrm{s}\mathbf{i}
+ \mathbf{L}_\mathrm{s}\dfrac{\mathrm{d}\mathbf{i}}{\mathrm{d}t}
+ \mathbf{v}
- \mathbf{e}
```

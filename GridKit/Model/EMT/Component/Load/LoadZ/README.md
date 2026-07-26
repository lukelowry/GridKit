# LoadZ Model

`LoadZ` represents an $N$-phase impedance load. Current $\mathbf{i}$ is injected
from the load into the EMT bus.

## Block Diagram

![LoadZ model block diagram](../../../../../../docs/Figures/EMT/LoadZ/diagram.png)

Figure 1: LoadZ model

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

Symbol | Description | Type | Order | JSON | Inputs | Outputs
------ | ----------- | ---- | ----- | ---- | ------ | -------
$\mathbf{z}$ | Impedance | [VectorFit](../../../Operators/Rational/VectorFit/README.md) | $NQ_{\mathbf{z}}$ | `Z` | $\mathbb{R}^N$ | $\mathbb{R}^N$

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
$\mathbf{i}$ | [A] | Current injection from load into EMT bus | $\mathbf{i} \in \mathbb{R}^N$, $\mathrm{rank}(\mathbf{E}^{\mathbf{z}})=N$

#### Algebraic

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{i}$ | [A] | Current injection from load into EMT bus | $\mathbf{i} \in \mathbb{R}^N$, $\mathbf{E}^{\mathbf{z}}=\mathbf{0}$

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
$\mathbf{v}$ | `v` | Input | [V] | Bus voltage at load port | $\mathbf{v} \in \mathbb{R}^N$
$\mathbf{i}$ | `i` | Output | [A] | Current injection at load port | $\mathbf{i} \in \mathbb{R}^N$

## Model Equations

### Differential Equations

For $\mathrm{rank}(\mathbf{E}^{\mathbf{z}})=N$,

```math
0 = \mathbf{z}[\mathbf{i}] + \mathbf{v}
```

### Algebraic Equations

For $\mathbf{E}^{\mathbf{z}}=\mathbf{0}$, the same residual is algebraic.

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
$\mathbf{w}_q$, $\mathbf{v}_q$ | `Z.w`, `Z.v` | Initial state | Impedance memory states

### Output Initialization

None.

## Monitors

Monitor | Units | Description | Note
------- | ----- | ----------- | ----
`i` | [A] | Load current injection | $\mathbf{i} \in \mathbb{R}^N$

## Development

The initial three-phase formulation fixes $N=3$ and requires
$Q_{\mathbf{z}}=0$, so the impedance reduces to a resistance and inductance
and $\mathbf{i}$ is a differential variable.

### Derived Parameters

```math
\mathbf{R} = \mathbf{D}^{\mathbf{z}},
\qquad
\mathbf{L} = \mathbf{E}^{\mathbf{z}}
```

### Differential Equations

```math
0 =
\mathbf{R}\mathbf{i}
+ \mathbf{L}\dfrac{\mathrm{d}\mathbf{i}}{\mathrm{d}t}
+ \mathbf{v}
```

### Temporary Interface

The implementation carries a required `enable` signal input that gates the bus
current injection. It is a placeholder for a physical
[Switch](../../Switch/README.md) and is not part of the model specification.

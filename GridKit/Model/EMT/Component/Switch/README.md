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

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{i}_{12}$ | [A] | Series current from terminal 1 to terminal 2 | $\mathbf{i}_{12} \in \mathbb{R}^3$

### External Variables

#### Differential

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{v}_1$ | [V] | Terminal 1 voltage owned by EMT bus | $\mathbf{v}_1 \in \mathbb{R}^3$
$\mathbf{v}_2$ | [V] | Terminal 2 voltage owned by EMT bus | $\mathbf{v}_2 \in \mathbb{R}^3$

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

### Input Initialization

The connected signal source supplies `open` before the harmonic network is
solved.

```math
\begin{aligned}
\mathbf{V}_r
  &\leftarrow \text{solved terminal RMS voltage phasor} \\
\mathbf{v}_r
  &\leftarrow \sqrt{2}\,\mathrm{Re}(\mathbf{V}_r) \\
\dfrac{\mathrm{d}\mathbf{v}_r}{\mathrm{d}t}
  &\leftarrow \sqrt{2}\,\mathrm{Re}(s_0\mathbf{V}_r),
  \quad r \in \{1,2\}.
\end{aligned}
```

### Internal Initialization

The assembled harmonic network supplies the switch-current phasor subject to

```math
\begin{cases}
\mathbf{I}_{12} = \mathbf{0}, & \text{open}, \\
\mathbf{V}_2-\mathbf{V}_1 = \mathbf{0}, & \text{closed}.
\end{cases}
```

The closed-position current cannot be recovered locally from the zero terminal-
voltage difference. At $t=0$,

```math
\begin{aligned}
\mathbf{I}_{12}
  &\leftarrow \text{solved switch RMS current phasor} \\
\mathbf{i}_{12}
  &\leftarrow \sqrt{2}\,\mathrm{Re}(\mathbf{I}_{12}) \\
\dfrac{\mathrm{d}\mathbf{i}_{12}}{\mathrm{d}t}
  &\leftarrow \sqrt{2}\,\mathrm{Re}(s_0\mathbf{I}_{12}).
\end{aligned}
```

### Output Initialization

```math
\begin{aligned}
\mathbf{I}_1
  &= -\mathbf{I}_{12} \\
\mathbf{I}_2
  &= \mathbf{I}_{12} \\
\mathbf{i}_r
  &\leftarrow \sqrt{2}\,\mathrm{Re}(\mathbf{I}_r) \\
\dfrac{\mathrm{d}\mathbf{i}_r}{\mathrm{d}t}
  &\leftarrow \sqrt{2}\,\mathrm{Re}(s_0\mathbf{I}_r),
  \quad r \in \{1,2\}.
\end{aligned}
```

## Monitors

Monitor | Units | Description | Note
------- | ----- | ----------- | ----
`open` | [-] | Switch command | Boolean; `true` open, `false` closed
`i12` | [A] | Series current from terminal 1 to terminal 2 | $\mathbf{i}_{12} \in \mathbb{R}^3$

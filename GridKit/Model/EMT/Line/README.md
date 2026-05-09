# EMT Transmission Line Model

The **EMT::Line** model represents the terminal characteristic-admittance
contribution of a three-phase transmission line in instantaneous abc
coordinates.

For this first pass, the model includes terminal characteristic admittance only.
Propagation functions, history buffers, and travel-time delay are future line
features.

## Model Parameters

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{Y}_c(s)$ | [p.u.] | Characteristic admittance rational approximation | `LineData::characteristic_admittance`

The characteristic admittance must have dimension three. Its rational
approximation parameters are documented in
[`RationalApprox`](../RationalApprox/README.md).

The full line model will also use a propagation function $\mathbf{H}(s)$ for
the remote-end history current. That term is not implemented in this pass.

## Model Variables

Let $S$ be the number of real realization states in $\mathbf{Y}_c(s)$.

### Internal Variables

#### Differential

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{x}_1$ | [-] | Terminal-1 characteristic-admittance memory states | Stored in `y_[0..S-1]`
$\mathbf{x}_2$ | [-] | Terminal-2 characteristic-admittance memory states | Stored in `y_[S..2*S-1]`

The state layout is

```text
y[0..S-1]       = terminal-1 characteristic-admittance memory states
y[S..2*S-1]     = terminal-2 characteristic-admittance memory states
```

#### Algebraic

None. Terminal currents are computed outputs, not line-owned solver variables.

### External Variables

#### Differential

None as line-owned variables. If $\mathbf{Y}_c(s)$ has nonzero derivative
feedthrough $\mathbf{E}$, `EMT::SystemModel` marks the connected bus voltage
variables as differential.

#### Algebraic

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{v}_1$ | [p.u.] | abc voltage vector at terminal 1 | Owned by the connected bus
$\mathbf{v}_2$ | [p.u.] | abc voltage vector at terminal 2 | Owned by the connected bus

## Model Equations

### Differential Equations

For each terminal $m \in \{1,2\}$, the internal memory
states are the `RationalApprox` realization states for the convolution
$(\mathbf{y}_c * \mathbf{v}_m)(t)$. The line does not add additional
line-specific differential equations. The expanded real-pole and complex-pair
equations are documented in [`RationalApprox`](../RationalApprox/README.md).

### Algebraic Equations

The ideal two-terminal convolutional line relation is

```math
\begin{aligned}
\mathbf{i}_1(t)
&= (\mathbf{y}_c * \mathbf{v}_1)(t) - (\mathbf{h} * \mathbf{i}_2)(t) \\
\mathbf{i}_2(t)
&= (\mathbf{y}_c * \mathbf{v}_2)(t) - (\mathbf{h} * \mathbf{i}_1)(t)
\end{aligned}
```

Until a fitted propagation function and history buffer are added, the
implementation uses the zero-delay approximation of the remote
propagation function:

```math
\begin{aligned}
\mathbf{i}_1(t)
&= (\mathbf{y}_c * \mathbf{v}_1)(t) - \mathbf{i}_2(t) \\
\mathbf{i}_2(t)
&= (\mathbf{y}_c * \mathbf{v}_2)(t) - \mathbf{i}_1(t)
\end{aligned}
```

For the current implementation this is evaluated in the equivalent reduced
form:

```math
\begin{aligned}
\mathbf{i}_1(t)
&= \left(\mathbf{y}_c * (\mathbf{v}_1 - \mathbf{v}_2)\right)(t) \\
\mathbf{i}_2(t)
&= \left(\mathbf{y}_c * (\mathbf{v}_2 - \mathbf{v}_1)\right)(t)
\end{aligned}
```

The terminal current $\mathbf{i}_m$ is oriented from the connected bus into the
line. Therefore its contribution entering that bus is $-\mathbf{i}_m$.

## Initialization

For each terminal $m$, the memory states $\mathbf{x}_m$ are initialized by
`RationalApprox` from the connected terminal voltage $\mathbf{v}_m(0)$ and
voltage derivative $\dot{\mathbf{v}}_m(0)$. The expanded initialization
formula is documented in [`RationalApprox`](../RationalApprox/README.md).

## Model Outputs

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{i}_1$ | [p.u.] | Terminal-1 line current | Positive from connected bus into line
$\mathbf{i}_2$ | [p.u.] | Terminal-2 line current | Positive from connected bus into line

Current injection into a bus has positive sign. Since line terminal currents are
oriented from bus into line, they appear with negative sign in the corresponding
bus current-balance residual.

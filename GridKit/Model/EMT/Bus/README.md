# EMT Bus Model

An EMT bus is a point of interconnection for three-phase electrical devices in
instantaneous abc coordinates. Each bus owns the phase voltage variables and the
current-balance residual rows.

The inherited GridKit state vector is named `y_` by convention. In this model,
the phase voltages are stored in `y_[0]`, `y_[1]`, and `y_[2]`.

**Sign Convention**

Currents entering the bus have positive sign in the bus current-balance
equation.

## Model Parameters

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$v_{a0}$ | [p.u.] | Initial phase-a voltage | `BusData::v0[0]`
$v_{b0}$ | [p.u.] | Initial phase-b voltage | `BusData::v0[1]`
$v_{c0}$ | [p.u.] | Initial phase-c voltage | `BusData::v0[2]`
$\dot{v}_{a0}$ | [p.u./s] | Initial phase-a voltage derivative | `BusData::vp0[0]`
$\dot{v}_{b0}$ | [p.u./s] | Initial phase-b voltage derivative | `BusData::vp0[1]`
$\dot{v}_{c0}$ | [p.u./s] | Initial phase-c voltage derivative | `BusData::vp0[2]`

## Model Variables

### Internal Variables

#### Differential

None by default. `SystemModel` may mark bus voltages as differential if a
connected component has derivative feedthrough with respect to terminal voltage.

#### Algebraic

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$v_a$ | [p.u.] | Phase-a voltage | Stored in `y_[0]`
$v_b$ | [p.u.] | Phase-b voltage | Stored in `y_[1]`
$v_c$ | [p.u.] | Phase-c voltage | Stored in `y_[2]`

### External Variables

#### Differential

None.

#### Algebraic

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{i}_{\eta \rightarrow b}$ | [p.u.] | Current contribution from connected terminal or device $\eta$ into bus $b$ | Assembled by `EMT::SystemModel`

## Model Equations

### Differential Equations

None.

### Algebraic Equations

The standalone bus contributes no current by itself. In an assembled EMT
network, let $\mathcal{C}(b)$ be the set of terminals and devices connected to
bus $b$. The bus current-balance residual is the sum of currents entering the
bus:

```math
\mathbf{0}
= \mathbf{r}_b(t)
= \sum_{\eta \in \mathcal{C}(b)} \mathbf{i}_{\eta \rightarrow b}(t)
```

The bus initializes this residual row to zero. Connected components add their
current contributions using the sign convention above.

## Initialization

The bus initializes phase voltages and their stored derivatives from
`BusData`:

```math
\begin{aligned}
v_a &= v_{a0} &
\dot{v}_a &= \dot{v}_{a0} \\
v_b &= v_{b0} &
\dot{v}_b &= \dot{v}_{b0} \\
v_c &= v_{c0} &
\dot{v}_c &= \dot{v}_{c0}
\end{aligned}
```

## Model Outputs

The bus voltage variables $v_a$, $v_b$, and $v_c$ are available to connected
component terminals through `EMT::SystemModel` connection binding.

# EMT Voltage Source Model

The **VoltageSource** model represents an ideal balanced three-phase voltage
source in instantaneous abc coordinates.

## Model Parameters

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$V_m$ | [p.u.] | Phase peak voltage | `VoltageSourceData::amplitude`
$f$ | [Hz] | Source frequency | `VoltageSourceData::frequency`
$\theta_0$ | [rad] | Initial phase-a angle | `VoltageSourceData::phase`
$\mathbf{i}_{s0}$ | [p.u.] | Initial source current | `VoltageSourceData::current0`

## Model Variables

### Internal Variables

#### Differential

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{v}_s$ | [p.u.] | Internal source voltage | Balanced abc oscillator state
$\mathbf{w}_s$ | [p.u./s] | Internal source voltage derivative | Balanced abc oscillator state

#### Algebraic

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{i}_s$ | [p.u.] | Source terminal current | Positive from connected bus into source

### External Variables

#### Differential

None.

#### Algebraic

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{v}$ | [p.u.] | Connected bus voltage | Owned by the connected bus

## Model Equations

### Differential Equations

None.

### Algebraic Equations

The source voltage is initialized as a balanced abc set:

```math
\begin{aligned}
v_{s,a}(t) &= V_m \sin(2\pi f t + \theta_0) \\
v_{s,b}(t) &= V_m \sin(2\pi f t + \theta_0 - 2\pi/3) \\
v_{s,c}(t) &= V_m \sin(2\pi f t + \theta_0 + 2\pi/3)
\end{aligned}
```

The source voltage then evolves as a harmonic oscillator:

```math
\begin{aligned}
\mathbf{0} &= \dot{\mathbf{v}}_s - \mathbf{w}_s \\
\mathbf{0} &= \dot{\mathbf{w}}_s + \omega^2 \mathbf{v}_s
\end{aligned}
```

where $\omega = 2\pi f$ and $\mathbf{w}_s$ is the internal source voltage
derivative state.

The source constrains the connected bus voltage:

```math
\mathbf{0} = \mathbf{v} - \mathbf{v}_s
```

The source terminal current is an algebraic unknown. It enters the connected
bus current-balance residual with negative sign because it is oriented from the
bus into the source.

## Initialization

The internal source voltage and derivative states are initialized from
`amplitude`, `frequency`, and `phase`. The source current is initialized from
`current0`. The connected bus voltage is made consistent with the source
voltage constraint by the system initialization.

## Model Outputs

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{i}_s$ | [p.u.] | Terminal source current | Positive from bus into source

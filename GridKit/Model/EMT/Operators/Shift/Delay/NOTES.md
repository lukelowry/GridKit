# Delay Model

`Delay` represents a scalar EMT delay operator using a rotating smooth current
history. The model maps terminal injected currents $I^{\text{inj}}_a$ and
$I^{\text{inj}}_b$ to incident currents $I^{\text{inc}}_a$ and
$I^{\text{inc}}_b$ through a two-way ring.

```math
\begin{aligned}
I^{\text{inc}}_b(t) &\approx I^{\text{inj}}_a(t-\tau) \\
I^{\text{inc}}_a(t) &\approx I^{\text{inj}}_b(t-\tau)
\end{aligned}
```

Note:
- The smooth selectors $A_n(t)$ and $B_n(t)$ correspond to terminals `a` and `b`.
- The smoothing scale $\mu$ is chosen so $\mu \gg 1/h$.

## Block Diagram

None.

## Model Parameters

Symbol | Units | JSON | Description | Note
------ | ----- | ---- | ----------- | ----
$\tau$ | [s] | `tau` | Total delay | Required, positive
$h$ | [s] | `h` | History resolution | Required, positive

### Parameter Validation

```math
\begin{aligned}
\tau &> 0 \\
h &> 0
\end{aligned}
```

### Model Derived Parameters

```math
\begin{aligned}
N &= 2\text{ceil}\left(\dfrac{\tau}{h}\right) \\
T &= \dfrac{2\tau}{N} \\
A_n(t) &= \sigma\left(\cos\left(\dfrac{\pi}{\tau}\left(t-\dfrac{2\tau n}{N}\right)\right)-\cos\left(\dfrac{\pi}{N}\right)\right) \\
B_n(t) &= \sigma\left(\cos\left(\dfrac{\pi}{\tau}\left(t-\tau-\dfrac{2\tau n}{N}\right)\right)-\cos\left(\dfrac{\pi}{N}\right)\right)
\end{aligned}
```

## Model Variables

### Internal Variables

#### Differential

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{I}$ | [-] | Rotating ring current states | $\mathbf{I}\in\mathbb{R}^N$

#### Algebraic

None.

### External Variables

#### Differential

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$I^{\text{inj}}_a$ | [-] | Terminal `a` injected current | $I^{\text{inj}}_a\in\mathbb{R}$
$I^{\text{inj}}_b$ | [-] | Terminal `b` injected current | $I^{\text{inj}}_b\in\mathbb{R}$

#### Algebraic

None.

## Model Ports

Symbol | Port | Type | Units | Description | Note
------ | ---- | ---- | ----- | ----------- | ----
$I^{\text{inj}}_a$ | `inj_a` | Input | [-] | Terminal `a` injected current port | $I^{\text{inj}}_a\in\mathbb{R}$
$I^{\text{inj}}_b$ | `inj_b` | Input | [-] | Terminal `b` injected current port | $I^{\text{inj}}_b\in\mathbb{R}$
$I^{\text{inc}}_a$ | `inc_a` | Output | [-] | Terminal `a` incident current port | $I^{\text{inc}}_a\in\mathbb{R}$
$I^{\text{inc}}_b$ | `inc_b` | Output | [-] | Terminal `b` incident current port | $I^{\text{inc}}_b\in\mathbb{R}$

## Model Equations

### Differential Equations

```math
\begin{aligned}
T\dot{I}_n
&=
A_n(t)\left(I^{\text{inj}}_a-I_n\right)
+
B_n(t)\left(I^{\text{inj}}_b-I_n\right)
\end{aligned}
```

### Algebraic Equations

None.

### Wiring

```math
\begin{aligned}
I^{\text{inc}}_a &= \sum_{n=0}^{N-1}A_n(t)I_n \\
I^{\text{inc}}_b &= \sum_{n=0}^{N-1}B_n(t)I_n
\end{aligned}
```

## Initialization

Initialize the ring states from the initial line-current history. The initial
state must be consistent with the terminal incident-current readouts:

```math
\begin{aligned}
I^{\text{inc}}_{a,0} &= \sum_{n=0}^{N-1}A_n(0)I_{n,0} \\
I^{\text{inc}}_{b,0} &= \sum_{n=0}^{N-1}B_n(0)I_{n,0}
\end{aligned}
```

For a quiescent initial line-current history, $I_{n,0}=0$.

## Monitors

None.

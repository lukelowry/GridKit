# Delay Model

`Delay` represents a smooth approximation of a transport delay on a scalar input
signal. The approximation uses a chain of $n$ identical first-order all-pass
Padé stages.

The Laplace domain representation is:

```math
e^{-s\tau}U(s) =
\lim_{n \to \infty}
\left(
\dfrac{1 - s\tau/(2n)}
      {1 + s\tau/(2n)}
\right)^n U(s)
```

The time domain convolutional form is:

```math
u(t-\tau) = \delta(t-\tau) * u(t)
```

## Block Diagram

<div align="center">
   <img align="center" src="../../../../../../docs/Figures/EMT/Delay/diagram.png">

  Figure 1: All-pass Padé-chain approximation of a pure delay
</div>

## Model Parameters

Symbol            | Units | JSON     | Description          | Typical Value | Note
------------------|-------|----------|----------------------|---------------|-----
$\tau$            | [s]   | `delay`  | Delay to approximate | --            | Required, positive
$\Delta t_{\min}$ | [s]   | `dt_min` | Block resolution     | --            | Required, positive

### Parameter Validation

```math
\begin{aligned}
\tau &> 0 \\
\Delta t_{\min} &> 0
\end{aligned}
```

### Model Derived Parameters

```math
n = \max\left(1,\text{floor}\left(\dfrac{\tau}{\Delta t_{\min}}\right)\right)
```

```math
h = \dfrac{\tau}{2n}
```

## Model Variables

### Internal Variables

#### Differential

Symbol       | Units | Description                                    | Note
------------ | ----- | ---------------------------------------------- | ----
$\mathbf{x}$ | [-]   | Lag-block state vector                         | $\mathbf{x} \in \mathbb{R}^n$

#### Algebraic

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$y$    | [-]   | Delayed output signal | $y \in \mathbb{R}$

### External Variables

#### Differential

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$u$    | [-]   | Input signal | $u \in \mathbb{R}$

#### Algebraic

None.

## Model Ports

Symbol | Port | Type | Units | Description | Note
------ | ---- | ---- | ----- | ----------- | ----
$u$ | `input` | Input | [-] | Input signal port | $u \in \mathbb{R}$
$y$ | `out` | Output | [-] | Output contribution port | $y \in \mathbb{R}$

## Model Equations

### Differential Equations

Each stage is a first-order all-pass Padé section:

```math
H_i(s) = \dfrac{1 - sh}{1 + sh},
\qquad h = \dfrac{\tau}{2n}.
```

The implementation uses one differential memory state per stage and a recursive
stage output. Let $v_0 = u$. For $i = 1,\ldots,n$:

```math
\begin{aligned}
0 &= -h\dot{x}_i - x_i + v_{i-1} \\
v_i &= -v_{i-1} + 2x_i
\end{aligned}
```

### Algebraic Equations

The delayed output is the final all-pass stage output:

```math
0 = -y + v_n
```

### Port Equations

```math
y = v_n
```

## Initialization

For an affine input $u(t) = u_0 + \dot{u}_0(t - t_0)$ at $t_0$, the chain is
initialized consistently with the stage delay:

```math
\begin{aligned}
x_i(t_0) &= u_0 - \left(i-\dfrac{1}{2}\right)\dfrac{\tau}{n}\dot{u}_0,
\qquad i = 1,\ldots,n \\
\dot{x}_i(t_0) &= \dot{u}_0,
\qquad i = 1,\ldots,n
\end{aligned}
```

The output therefore starts from the affine delayed value:

```math
y_0 = x_n(t_0) = u_0 - \tau\dot{u}_0
```

## Monitors

None.

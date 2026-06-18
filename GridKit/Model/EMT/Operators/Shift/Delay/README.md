# Delay Model

`Delay` represents a smooth approximation of a transport delay on a scalar input
signal. The approximation uses a chain of $n$ identical first-order lag stages.

The Laplace domain representation is:

```math
e^{-s\tau}U(s) =
\lim_{n \to \infty}\left(\dfrac{1}{1 + s\tau/n}\right)^n U(s)
```

The time domain convolutional form is:

```math
u(t-\tau) = \delta(t-\tau) * u(t)
```

## Block Diagram

<div align="center">
   <img align="center" src="../../../../../../docs/Figures/EMT/Delay/diagram.png">

  Figure 1: Lag-chain approximation of a pure delay
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
\mathbf{b} =
\begin{bmatrix}
1 & 0 & \cdots & 0
\end{bmatrix}^{\mathsf T}
```

```math
\mathbf{A} =
\begin{bmatrix}
-1 & 0  & 0  & \cdots & 0 \\
 1 & -1 & 0  & \cdots & 0 \\
 0 & 1  & -1 & \ddots & \vdots \\
\vdots & \ddots & \ddots & \ddots & 0 \\
0 & \cdots & 0 & 1 & -1
\end{bmatrix}
```

## Model Variables

### Internal Variables

#### Differential

Symbol       | Units | Description                                    | Note
------------ | ----- | ---------------------------------------------- | ----
$\mathbf{x}$ | [-]   | Lag-block state vector                         | $\mathbf{x} \in \mathbb{R}^n$

#### Algebraic

None.

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

The lag-chain residual is:

```math
0 = -\tau\,\dot{\mathbf{x}} + n\left(\mathbf{A}\mathbf{x}+\mathbf{b}u\right)
```

### Algebraic Equations

None.

### Port Equations

```math
y = x_n
```

## Initialization

For a constant input $u_0$ at $t_0$, the chain is at rest:

```math
\begin{aligned}
x_1(t_0) = x_2(t_0) = \cdots = x_n(t_0) &= u_0 \\
\dot{x}_1(t_0) = \dot{x}_2(t_0) = \cdots = \dot{x}_n(t_0) &= 0
\end{aligned}
```

A steady input therefore passes through unchanged at $t_0$ and downstream
consumers initialize consistently:

```math
y_0 = x_n(t_0)
```

## Monitors

None.

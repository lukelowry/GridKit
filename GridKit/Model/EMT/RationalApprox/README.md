# Rational Approximation Equation Block

The **RationalApprox** block represents a square vector-fitted matrix rational
approximation with real poles and complex conjugate pole pairs. It owns fitted
coefficients and realization math only. It is not a GridKit component and does
not own solver variables, signals, ports, or global indices.

## Model Parameters

For vector dimension $N$, real pole count $M$, and complex-pair count $Q$:

Symbol | Units | Description | Storage
------ | ----- | ----------- | -------
$\mathbf{D}$ | [-] | Direct-feedthrough matrix | Row-major `N x N`
$\mathbf{E}$ | [s] | Input-derivative matrix | Row-major `N x N`
$p_k$ | [1/s] | Real vector-fitting pole | Length `M`
$\mathbf{b}_k$ | [-] | Real-pole input coupling vector | Mode-major `M x N`
$\mathbf{c}_k$ | [1/s] | Real-pole output residue vector | Mode-major `M x N`
$a_q + j\omega_q$ | [1/s] | Complex-pair pole with $\omega_q > 0$ | One stored pole per pair
$\mathbf{b}_{q,r}$ | [-] | Real part of complex-pair input coupling vector | Mode-major `Q x N`
$\mathbf{b}_{q,i}$ | [-] | Imaginary part of complex-pair input coupling vector | Mode-major `Q x N`
$\mathbf{c}_{q,r}$ | [1/s] | Real part of complex-pair output residue vector | Mode-major `Q x N`
$\mathbf{c}_{q,i}$ | [1/s] | Imaginary part of complex-pair output residue vector | Mode-major `Q x N`

## Model Variables

`RationalApprox` does not own variables. The owning model supplies input,
input-derivative, memory-state, memory-state-derivative, output, residual, and
Jacobian-entry storage.

### Internal Variables

#### Differential

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$x_k$ | [-] | Real-pole memory state | One state per real pole
$x_{q,r}$ | [-] | Complex-pair real memory state | One real state per complex pair
$x_{q,i}$ | [-] | Complex-pair imaginary memory state | One imaginary state per complex pair

#### Algebraic

None.

### External Variables

#### Differential

None. The owning model supplies $\dot{\mathbf{u}}$ when derivative feedthrough
is present.

#### Algebraic

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{u}$ | [-] | Input vector | Supplied by owning model

## Model Equations

The rational approximation is represented in pole form:

```math
\mathbf{F}(s) \approx \mathbf{D} + s\mathbf{E}
  + \sum_{k=1}^{M} \frac{\mathbf{c}_k \mathbf{b}_k^T}{s - p_k}
  + \sum_{q=1}^{Q}
    \left(
      \frac{\mathbf{c}_q \mathbf{b}_q^T}{s - (a_q + j\omega_q)}
      + \frac{\mathbf{c}_q^* \mathbf{b}_q^{*T}}{s - (a_q - j\omega_q)}
    \right)
```

### Differential Equations

For each real pole,

```math
\dot{x}_k = \mathbf{b}_k^T\mathbf{u} + p_k x_k
```

For each stored complex pole $a_q + j\omega_q$,

```math
\begin{aligned}
\dot{x}_{q,r} &= \mathbf{b}_{q,r}^T\mathbf{u}
  + a_q x_{q,r} - \omega_q x_{q,i} \\
\dot{x}_{q,i} &= \mathbf{b}_{q,i}^T\mathbf{u}
  + \omega_q x_{q,r} + a_q x_{q,i}
\end{aligned}
```

### Algebraic Equations

The output equation is

```math
\mathbf{z} = \mathbf{D}\mathbf{u} + \mathbf{E}\dot{\mathbf{u}}
  + \sum_{k=1}^{M}\mathbf{c}_k x_k
  + 2\sum_{q=1}^{Q}(\mathbf{c}_{q,r}x_{q,r} - \mathbf{c}_{q,i}x_{q,i})
```

The state residual equations provided to owning models are

```math
\begin{aligned}
0 &= -\dot{x}_k + \mathbf{b}_k^T\mathbf{u} + p_k x_k \\
0 &= -\dot{x}_{q,r} + \mathbf{b}_{q,r}^T\mathbf{u}
     + a_q x_{q,r} - \omega_q x_{q,i} \\
0 &= -\dot{x}_{q,i} + \mathbf{b}_{q,i}^T\mathbf{u}
     + \omega_q x_{q,r} + a_q x_{q,i}
\end{aligned}
```

## Initialization

For an affine initial input trajectory, real-pole memory states are initialized
as

```math
\begin{aligned}
x_{k0} &= -\frac{\mathbf{b}_k^T\mathbf{u}_0}{p_k}
          -\frac{\mathbf{b}_k^T\dot{\mathbf{u}}_0}{p_k^2} \\
\dot{x}_{k0} &= \mathbf{b}_k^T\mathbf{u}_0 + p_k x_{k0}
\end{aligned}
```

For complex conjugate pole pairs, the block applies the same steady-state
expression to the stored positive-imaginary pole:

```math
x_{q0} =
-\frac{\mathbf{b}_q^T\mathbf{u}_0}{a_q + j\omega_q}
-\frac{\mathbf{b}_q^T\dot{\mathbf{u}}_0}{(a_q + j\omega_q)^2}
```

and stores $\operatorname{Re}(x_{q0})$ and $\operatorname{Im}(x_{q0})$ in
$x_{q,r}$ and $x_{q,i}$.

## Model Outputs

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{z}$ | [-] | Rational approximation output vector | Written to caller-provided storage

## Notes

- Complex conjugate pole pairs are stored once, using the positive-imaginary
  pole, and expanded into real two-state blocks.
- A model with zero real poles and zero complex pairs is valid.
- Real poles must be nonzero.
- Complex-pair imaginary parts must be positive.

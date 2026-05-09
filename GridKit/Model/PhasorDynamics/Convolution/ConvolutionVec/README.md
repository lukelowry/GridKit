# ConvolutionVec

The **ConvolutionVec** model is a square MIMO helper component for representing
real vector-fitting convolution operators with common real poles, complex
conjugate pole pairs, and low-rank modal coupling.

For vector dimension `N`, real pole count `M`, and complex-pair count `Q`, the
inherited GridKit state vector stores variables in the order all local input
proxies `u_i`, all outputs `z_i`, one memory state `x_k` per real pole, then
two real memory states `x_{q,r}`, `x_{q,i}` per complex pair.

## Model Parameters

Symbol | Units | Description | Storage
------ | ----- | ----------- | -------
$\mathbf{D}$ | [-] | Direct-feedthrough matrix | Row-major `N x N`
$\mathbf{E}$ | [s] | Input-derivative matrix | Row-major `N x N`
$p_k$ | [1/s] | Vector-fitting pole | One real pole per mode
$\mathbf{b}_k$ | [-] | Input coupling vector | Mode-major `M x N`
$\mathbf{c}_k$ | [1/s] | Output residue vector | Mode-major `M x N`
$a_q + j\omega_q$ | [1/s] | Complex-pair pole with $\omega_q > 0$ | One stored pole per conjugate pair
$\mathbf{b}_{q,r}, \mathbf{b}_{q,i}$ | [-] | Real and imaginary input coupling vectors | Mode-major `Q x N`
$\mathbf{c}_{q,r}, \mathbf{c}_{q,i}$ | [1/s] | Real and imaginary output residue vectors | Mode-major `Q x N`
$\mathbf{u}_0$ | [-] | Initial input vector | Length `N`
$\dot{\mathbf{u}}_0$ | [-/s] | Initial input derivative vector | Length `N`

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

The state realization is

```math
\dot{x}_k = \mathbf{b}_k^T \mathbf{u} + p_k x_k
```

and for each complex pair is expanded into real states:

```math
\begin{aligned}
\dot{x}_{q,r} &= \mathbf{b}_{q,r}^T\mathbf{u}
  + a_q x_{q,r} - \omega_q x_{q,i} \\
\dot{x}_{q,i} &= \mathbf{b}_{q,i}^T\mathbf{u}
  + \omega_q x_{q,r} + a_q x_{q,i}
\end{aligned}
```

with output

```math
\mathbf{z} = \mathbf{D}\mathbf{u} + \mathbf{E}\dot{\mathbf{u}}
  + \sum_{k=1}^{M} \mathbf{c}_k x_k
  + 2\sum_{q=1}^{Q}(\mathbf{c}_{q,r}x_{q,r} - \mathbf{c}_{q,i}x_{q,i})
```

The residual equations are

```math
\begin{aligned}
0 &= u_i - U_i \\
0 &= z_i - \sum_j \mathbf{D}_{ij} u_j - \sum_j \mathbf{E}_{ij}\dot{u}_j
     - \sum_k c_{ki} x_k
     - 2\sum_q(c_{q,r,i}x_{q,r} - c_{q,i,i}x_{q,i}) \\
0 &= -\dot{x}_k + \sum_i b_{ki}u_i + p_k x_k \\
0 &= -\dot{x}_{q,r} + \sum_i b_{q,r,i}u_i
     + a_q x_{q,r} - \omega_q x_{q,i} \\
0 &= -\dot{x}_{q,i} + \sum_i b_{q,i,i}u_i
     + \omega_q x_{q,r} + a_q x_{q,i}
\end{aligned}
```

## Initialization

For an affine initial input trajectory, the memory-state initial conditions are

```math
\begin{aligned}
x_{k0} &= -\frac{\mathbf{b}_k^T \mathbf{u}_0}{p_k}
          - \frac{\mathbf{b}_k^T \dot{\mathbf{u}}_0}{p_k^2} \\
\dot{x}_{k0} &= \mathbf{b}_k^T \mathbf{u}_0 + p_k x_{k0}
\end{aligned}
```

For complex pairs, the component applies the same steady-state expression to
the stored positive-imaginary pole and then stores the real and imaginary parts:

```math
x_{q0} = -\frac{\mathbf{b}_q^T \mathbf{u}_0}{a_q + j\omega_q}
         - \frac{\mathbf{b}_q^T \dot{\mathbf{u}}_0}{(a_q + j\omega_q)^2}
```

The output is initialized consistently:

```math
\mathbf{z}_0 = \mathbf{D}\mathbf{u}_0 + \mathbf{E}\dot{\mathbf{u}}_0
  + \sum_{k=1}^{M} \mathbf{c}_k x_{k0}
  + 2\sum_{q=1}^{Q}(\mathbf{c}_{q,r}x_{q,r,0} - \mathbf{c}_{q,i}x_{q,i,0})
```

## Notes

- The component stores only real solver states; complex pairs are expanded into
  real two-state blocks.
- Input and output dimensions are square and must match `dimension`.
- A model with no real modes and no complex pairs is valid and contains only `u`
  and `z`.

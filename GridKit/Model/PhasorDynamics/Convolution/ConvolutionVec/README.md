# ConvolutionVec

The **ConvolutionVec** model is a square MIMO helper component for representing
real vector-fitting convolution operators with common poles and low-rank modal
coupling.

For vector dimension `N` and pole count `M`, the inherited GridKit state vector
stores variables in the order all local input proxies `u_i`, all outputs `z_i`,
then one memory state `x_k` per pole.

## Model Parameters

Symbol | Units | Description | Storage
------ | ----- | ----------- | -------
$\mathbf{D}$ | [-] | Direct-feedthrough matrix | Row-major `N x N`
$\mathbf{E}$ | [s] | Input-derivative matrix | Row-major `N x N`
$p_k$ | [1/s] | Vector-fitting pole | One real pole per mode
$\mathbf{b}_k$ | [-] | Input coupling vector | Mode-major `M x N`
$\mathbf{c}_k$ | [1/s] | Output residue vector | Mode-major `M x N`
$\mathbf{u}_0$ | [-] | Initial input vector | Length `N`
$\dot{\mathbf{u}}_0$ | [-/s] | Initial input derivative vector | Length `N`

## Model Equations

The rational approximation is represented in pole form:

```math
\mathbf{F}(s) \approx \mathbf{D} + s\mathbf{E}
  + \sum_{k=1}^{M} \frac{\mathbf{c}_k \mathbf{b}_k^T}{s - p_k}
```

The state realization is

```math
\dot{x}_k = \mathbf{b}_k^T \mathbf{u} + p_k x_k
```

with output

```math
\mathbf{z} = \mathbf{D}\mathbf{u} + \mathbf{E}\dot{\mathbf{u}}
  + \sum_{k=1}^{M} \mathbf{c}_k x_k
```

The residual equations are

```math
\begin{aligned}
0 &= u_i - U_i \\
0 &= z_i - \sum_j \mathbf{D}_{ij} u_j - \sum_j \mathbf{E}_{ij}\dot{u}_j
     - \sum_k c_{ki} x_k \\
0 &= -\dot{x}_k + \sum_i b_{ki}u_i + p_k x_k
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

The output is initialized consistently:

```math
\mathbf{z}_0 = \mathbf{D}\mathbf{u}_0 + \mathbf{E}\dot{\mathbf{u}}_0
  + \sum_{k=1}^{M} \mathbf{c}_k x_{k0}
```

## Notes

- Real coefficients only; complex-pair support is future work.
- Input and output dimensions are square and must match `dimension`.
- A zero-pole-count model is valid and contains only `u` and `z`.
- Parser, SystemModel, examples, and CSV source integration are out of scope for this pass.

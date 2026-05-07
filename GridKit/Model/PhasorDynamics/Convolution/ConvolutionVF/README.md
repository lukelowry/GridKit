# ConvolutionVF

The **ConvolutionVF** model is a scalar helper component for representing a
convolution operator using real vector-fitting coefficients.

The inherited GridKit state vector is named `y_` by convention. In this model,
the physical convolution output is named `z` and is stored in `y_[1]`. The
local state `u` is a differential proxy for the input signal because the term
`e * dot(u)` requires an input derivative while signals currently expose values
only.

## Model Parameters

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$d$ | [-] | Direct-feedthrough coefficient |
$e$ | [s] | Input-derivative coefficient |
$p_n$ | [1/s] | Vector-fitting pole | Real pole form
$r_n$ | [-] | Vector-fitting residue | One residue per pole
$u_0$ | [-] | Initial input value |
$\dot{u}_0$ | [-/s] | Initial input derivative |

## Model Variables

### Internal Variables

#### Differential

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$u$ | [-] | Local input proxy | Stored in `y_[0]`
$x_n$ | [-] | Vector-fitting memory state | Stored in `y_[2 + n]`

#### Algebraic

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$z$ | [-] | Convolution output | Stored in `y_[1]`

### External Variables

#### Differential

None.

#### Algebraic

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$U$ | [-] | Input signal | Constrained to local state $u$

## Model Equations

The rational approximation is represented in pole form:

```math
f_c(s) \approx d + e s + \sum_{n=1}^{N}\frac{r_n}{s - p_n}
```

### Differential Equations

```math
\dot{x}_n = u + p_n x_n
```

### Algebraic Equations

```math
\begin{aligned}
0 &= u - U \\
0 &= z - d u - e\dot{u} - \sum_{n=1}^{N} r_n x_n
\end{aligned}
```

## Initialization

The model is initialized from $u_0$ and $\dot{u}_0$. For an affine initial input
trajectory, the memory-state initial conditions are

```math
\begin{aligned}
x_{n0} &= -\frac{u_0}{p_n} - \frac{\dot{u}_0}{p_n^2} \\
\dot{x}_{n0} &= u_0 + p_n x_{n0}
\end{aligned}
```

The output is initialized consistently:

```math
z_0 = d u_0 + e\dot{u}_0 + \sum_{n=1}^{N} r_n x_{n0}
```

## Notes

- Real coefficients only.
- Pole and residue vectors must have equal length.
- Poles must be nonzero.
- A zero-pole-count model is valid and contains only `u` and `z`.
- Parser, SystemModel, input-format, monitor, and test integration are out of
  scope for this component skeleton.

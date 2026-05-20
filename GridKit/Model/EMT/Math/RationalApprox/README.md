# RationalApprox Model

`RationalApprox` represents a vector-fitted rational matrix approximation as a
real time-domain realization. EMT components use this model as a reusable
building block for convolution outputs such as characteristic admittance current
and propagation-function history current. Travel-time delay and history
interpolation are separate line-model concerns.

For a real input vector $\mathbf{u}(t)$, the model approximates a square matrix
function in pole-residue form:

``` math
\begin{aligned}
\mathbf{F}(s)
  &\approx
  \mathbf{D} + s\mathbf{E}
  + \sum_{m=1}^{M}\frac{\mathbf{R}_m}{s-p_m}
  + \sum_{q=1}^{Q}
    \left(
      \frac{\mathbf{R}_q}{s-\lambda_q}
      + \frac{\mathbf{R}_q^*}{s-\lambda_q^*}
    \right)
\end{aligned}
```

## Model Parameters

For vector dimension $N$, real pole count $M$, and complex-pair count $Q$:

Symbol | Units | Description | Note
-------|-------|-------------|---------------------------------
$\mathbf{D}$ | [-] | Direct-feedthrough matrix | $\mathbb{R}^{N \times N}$
$\mathbf{E}$ | [s] | Input-derivative feedthrough matrix | $\mathbb{R}^{N \times N}$
$p_m$ | [1/s] | Real vector-fitting pole | $p_m \neq 0$
$\mathbf{R}_m$ | [1/s] | Real-pole residue matrix | $\mathbb{R}^{N \times N}$
$a_q + j\omega_q$ | [1/s] | Complex pole stored with positive imaginary part | $\omega_q > 0$
$\mathbf{R}_{q,r}$ | [1/s] | Real part of complex-pair residue matrix | $\mathbb{R}^{N \times N}$
$\mathbf{R}_{q,i}$ | [1/s] | Imaginary part of complex-pair residue matrix | $\mathbb{R}^{N \times N}$

## Model Derived Parameters

For each complex conjugate pole pair,

``` math
\begin{aligned}
\lambda_q &= a_q + j\omega_q \\
\mathbf{R}_q &= \mathbf{R}_{q,r} + j\mathbf{R}_{q,i}
\end{aligned}
```

Each real pole contributes one $N$-component memory state. Each complex
conjugate pole pair contributes two real $N$-component memory states. The total
number of scalar differential states used by the realization is

``` math
S = N(M + 2Q)
```

## Model Variables

`RationalApprox` is a reusable equation block and does not own electrical
ports. The variables below are owned by the EMT component that uses the
block.

### Internal Variables

#### Differential

Symbol | Units | Description | Note
-------|-------|-------------|---------------------------------
$\mathbf{x}_m$ | [-] | Real-pole memory state | $\mathbf{x}_m \in \mathbb{R}^N$
$\mathbf{x}_{q,r}$ | [-] | Real part of complex-pair memory state | $\mathbf{x}_{q,r} \in \mathbb{R}^N$
$\mathbf{x}_{q,i}$ | [-] | Imaginary part of complex-pair memory state | $\mathbf{x}_{q,i} \in \mathbb{R}^N$

#### Algebraic

None.

### External Variables

External variables enter model equations but are owned by other components. For
this equation block, the owning EMT component supplies the input vector.

#### Differential

Symbol | Units | Description | Note
-------|-------|-------------|---------------------------------
$\mathbf{u}$ | [-] | Input vector | $\dot{\mathbf{u}}$ is required when $\mathbf{E} \neq \mathbf{0}$

#### Algebraic

None.

## Model Equations

The equations below define the real time-domain realization of the rational
approximation.

### Differential Equations

For each real pole $p_m$,

``` math
0 = -\dot{\mathbf{x}}_m + \mathbf{u} + p_m\mathbf{x}_m
```

For each complex conjugate pole pair $\lambda_q = a_q + j\omega_q$,

``` math
\left\{
\begin{aligned}
0 &=
  -\dot{\mathbf{x}}_{q,r}
  + \mathbf{u}
  + a_q\mathbf{x}_{q,r}
  - \omega_q\mathbf{x}_{q,i} \\
0 &=
  -\dot{\mathbf{x}}_{q,i}
  + \omega_q\mathbf{x}_{q,r}
  + a_q\mathbf{x}_{q,i}
\end{aligned}
\right.
```

### Algebraic Equations

The convolution output is

``` math
\begin{aligned}
\mathbf{z}
  &=
  \mathbf{D}\mathbf{u}
  + \mathbf{E}\dot{\mathbf{u}}
  + \sum_{m=1}^{M}\mathbf{R}_m\mathbf{x}_m
  + 2\sum_{q=1}^{Q}
    \left(
      \mathbf{R}_{q,r}\mathbf{x}_{q,r}
      - \mathbf{R}_{q,i}\mathbf{x}_{q,i}
    \right)
\end{aligned}
```

## Initialization

For an affine initial input trajectory,

``` math
\mathbf{u}(t) \approx \mathbf{u}_0 + t\dot{\mathbf{u}}_0
```

the real-pole memory states are initialized as

``` math
\begin{aligned}
\mathbf{x}_{m0}
  &=
  -\frac{\mathbf{u}_0}{p_m}
  - \frac{\dot{\mathbf{u}}_0}{p_m^2} \\
\dot{\mathbf{x}}_{m0}
  &=
  \mathbf{u}_0 + p_m\mathbf{x}_{m0}
\end{aligned}
```

For each complex conjugate pole pair, first compute the complex initial state
and derivative

``` math
\begin{aligned}
\mathbf{x}_{q0}
  &=
  -\frac{\mathbf{u}_0}{\lambda_q}
  - \frac{\dot{\mathbf{u}}_0}{\lambda_q^2} \\
\dot{\mathbf{x}}_{q0}
  &=
  \mathbf{u}_0 + \lambda_q\mathbf{x}_{q0}
\end{aligned}
```

and then set

``` math
\begin{aligned}
\mathbf{x}_{q,r,0} &= \operatorname{Re}(\mathbf{x}_{q0}) \\
\mathbf{x}_{q,i,0} &= \operatorname{Im}(\mathbf{x}_{q0}) \\
\dot{\mathbf{x}}_{q,r,0} &= \operatorname{Re}(\dot{\mathbf{x}}_{q0}) \\
\dot{\mathbf{x}}_{q,i,0} &= \operatorname{Im}(\dot{\mathbf{x}}_{q0})
\end{aligned}
```

The initial output is

``` math
\begin{aligned}
\mathbf{z}_0
  &=
  \mathbf{D}\mathbf{u}_0
  + \mathbf{E}\dot{\mathbf{u}}_0
  + \sum_{m=1}^{M}\mathbf{R}_m\mathbf{x}_{m0} \\
  &\quad
  + 2\sum_{q=1}^{Q}
  \left(
    \mathbf{R}_{q,r}\mathbf{x}_{q,r,0}
    - \mathbf{R}_{q,i}\mathbf{x}_{q,i,0}
  \right)
\end{aligned}
```

## Model Outputs

Symbol | Units | Description | Note
-------|-------|-------------|---------------------------------
$\mathbf{z}$ | [-] | Rational approximation output vector | $\mathbf{z} \in \mathbb{R}^N$

For characteristic admittance convolution, $\mathbf{z}$ is interpreted as a
port current contribution. For propagation-function convolution,
$\mathbf{z}$ is interpreted as a propagated history-current contribution.

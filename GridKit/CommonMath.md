# CommonMath

Smooth, autodiff-friendly replacements for piecewise functions used across GridKit component models. See [CommonMath.hpp](CommonMath.hpp) for implementation details.

## Primitives

| Name | Exact Target | Smooth Approximation | Description |
|------|--------------|----------------------|-------------|
| `sigmoid` | $H(x)$ | $\sigma(x) = \frac{1}{1+\exp(-\mu x)}$ | Step function |
| `ramp` | $\max(x,0)$ | $\rho(x)=\mu^{-1}\log(1+\exp(\mu x))$ | Smooth one-sided ramp |

The scale $\mu=240$ is chosen so $\sigma$ behaves like a step on inputs of order 1 while keeping derivatives finite. As $\mu \to \infty$, these functions approach their exact targets.

Softplus avoids the negative undershoot of $x\sigma(x)$, with a small positive breakpoint bias:

```math
\rho(0)=\frac{\log 2}{\mu}.
```

## Derived Functions

| Name | Exact Target | Smooth Approximation | Description |
|------|--------------|----------------------|-------------|
| `clamp` | $\min(\max(x,\ell),u)$ | $\ell + \rho(x-\ell) - \rho(x-u)$ | Bounded saturation |
| `slew` | $\min(\max(f,-r),r)$ | $-r + \rho(f+r) - \rho(f-r)$ | Symmetric slew-rate limiter |
| `rampsat` | $h\,\operatorname{clamp}\!\left(\frac{x-a}{b-a},0,1\right)$ | $\frac{h}{b-a}\left[\rho(x-a)-\rho(x-b)\right]$ | Saturating linear ramp |

`rampsat` is a monotone saturating linear ramp, implemented as the difference of two smooth ramps:

```math
\operatorname{rampsat}(x;\,a,b,h)
=
\frac{h}{b-a}\left[\rho(x-a)-\rho(x-b)\right].
```

It is not a signal-processing window function.

## Anti-Windup

For a limited state $x \in [x_{\min}, x_{\max}]$, the indicators are:

| Name | Exact Target | Smooth Approximation | Description |
|------|--------------|----------------------|-------------|
| $\phi_L$ | $H(x-x_{\min})$ | $\sigma(x-x_{\min})$ | Above-lower-limit indicator |
| $\phi_U$ | $H(x_{\max}-x)$ | $\sigma(x_{\max}-x)$ | Below-upper-limit indicator |
| $\phi_0$ | $\begin{cases}1 & x_{\min}<x<x_{\max}\\0 & \text{else}\end{cases}$ | $\phi_L+\phi_U-1$ | Interior pulse indicator |

For a pre-limit derivative $f$, the exact anti-windup rule is:

```math
\dot x =
   \begin{cases}
      f
         &  \text{if } (x_{\min} < x < x_{\max}) & \lor \\
         &  \quad (x \leq x_{\min} \land f > 0)  & \lor \\
         &  \quad (x \geq x_{\max} \land f < 0)         \\
      0  &  \text{else}
   \end{cases}
```

GridKit uses the smooth gate $\dot x = \phi(x,f)f$, where

```math
\phi(x, f) = \phi_L \phi_U + (1 - \phi_U)\,\sigma(-f) + (1 - \phi_L)\,\sigma(f).
```

The first term passes interior dynamics. The second and third terms pass restoring motion from the upper and lower limits, respectively; otherwise, $\phi$ smoothly blocks windup.

## Model Usage

Ramp (`Math::ramp`, $\rho$):

- [IEEET1](Model/PhasorDynamics/Exciter/IEEET1/README.md): smooth magnetic saturation above the saturation knee
- REGCA: applies reactive-current and active-current rate-limit corrections

Saturating ramp (`Math::rampsat`, $\operatorname{rampsat}$):

- REGCA: defines the LVPL and LVACM piecewise-linear curves

Anti-windup gate (`Math::indicator`):

- [IEEET1](Model/PhasorDynamics/Exciter/IEEET1/README.md): gates $\dot V_R$ on $V_R \in (V_{rmin}, V_{rmax})$
- [TGOV1](Model/PhasorDynamics/Governor/Tgov1/README.md): gates $\dot P_v$ on $P_v \in (P_{vmin}, P_{vmax})$
- [SEXS-PTI](Model/PhasorDynamics/Exciter/SEXS-PTI/README.md): gates $\dot E_{fd}$ on $E_{fd} \in (E_{fd,\min}, E_{fd,\max})$

Window gate (`Math::sigmoid((x - lower)(upper - x) / (upper - lower))`):

- [IEEEST](Model/PhasorDynamics/Stabilizer/IEEEST/README.md): gates $V_s$ by the $V_{ct}$ cutout window $[V_{cl}, V_{cu}]$

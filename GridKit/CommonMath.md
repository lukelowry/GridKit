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
| `max` | $\begin{cases}x & x>y\\y & x\le y\end{cases}$ | $y+\rho(x-y)$ | Smooth binary maximum |
| `min` | $\begin{cases}x & x<y\\y & x\ge y\end{cases}$ | $x-\rho(x-y)$ | Smooth binary minimum |
| `clamp` | $\begin{cases}\ell & x<\ell\\x & \ell\le x\le u\\u & x>u\end{cases}$ | $\ell + \rho(x-\ell) - \rho(x-u)$ | Bounded saturation |
| `deadband` | $\begin{cases}x-\ell & x<\ell\\0 & \ell\le x\le u\\x-u & x>u\end{cases}$ | $\rho(x-u)-\rho(\ell-x)$ | Signed two-sided deadband |
| `slew` | $\begin{cases}-r & f<-r\\f & -r\le f\le r\\r & f>r\end{cases}$ | $-r + \rho(f+r) - \rho(f-r)$ | Symmetric slew-rate limiter |
| `linseg` | $\begin{cases}0 & x<a\\\frac{h}{b-a}(x-a) & a\le x\le b\\h & x>b\end{cases}$ | $\frac{h}{b-a}\left[\rho(x-a)-\rho(x-b)\right]$ | Saturated linear segment contribution |
| `above` | $\begin{cases}0 & x\le x_{\min}\\1 & x>x_{\min}\end{cases}$ | $\sigma(x-x_{\min})$ | Above-lower-limit indicator |
| `below` | $\begin{cases}1 & x<x_{\max}\\0 & x\ge x_{\max}\end{cases}$ | $\sigma(x_{\max}-x)$ | Below-upper-limit indicator |
| `inside` | $\begin{cases}1 & x_{\min}<x<x_{\max}\\0 & \text{else}\end{cases}$ | $\sigma(x-x_{\min})+\sigma(x_{\max}-x)-1$ | Interior pulse indicator |
| `outside` | $\begin{cases}1 & x<x_{\min} \lor x>x_{\max}\\0 & \text{else}\end{cases}$ | $\sigma(x_{\min}-x)+\sigma(x-x_{\max})$ | Outside-band indicator |
| `antiwindup` | $\begin{cases}f & x_{\min}<x<x_{\max}\\f & x\le x_{\min}\land f>0\\f & x\ge x_{\max}\land f<0\\0 & \text{otherwise}\end{cases}$ | $\phi(x,f)f$ | Anti-windup limited derivative |

Binary `min` and `max` inherit the ramp breakpoint bias:

```math
\text{max}(x,x)=x+\rho(0), \qquad
\text{min}(x,x)=x-\rho(0).
```

`deadband` is the signed complement of `clamp`, implemented as the difference
of the upper and lower one-sided deadband ramps:

```math
\text{deadband}(x;\,\ell,u)=\rho(x-u)-\rho(\ell-x).
```

Callers should supply $\ell \le u$.

`linseg` is a saturated linear segment contribution, implemented as the
difference of two smooth ramps:

```math
\text{linseg}(x;\,a,b,h)
=
\frac{h}{b-a}\left[\rho(x-a)-\rho(x-b)\right].
```

Callers should supply $a < b$. The height $h$ may be positive or negative. It
is not a signal-processing window function.

## Anti-Windup Indicator

For a limited state $x \in [x_{\min}, x_{\max}]$, define
$\phi_L=\text{above}(x,x_{\min})$ and
$\phi_U=\text{below}(x,x_{\max})$.

GridKit's `indicator` function is the smooth anti-windup gate $\phi(x,f)$,
where

```math
\phi(x, f) = \phi_L \phi_U + (1 - \phi_U)\,\sigma(-f) + (1 - \phi_L)\,\sigma(f).
```

The first term passes interior dynamics. The second and third terms pass restoring motion from the upper and lower limits, respectively; otherwise, $\phi$ smoothly blocks windup.

## Model Usage

Ramp (`Math::ramp`, $\rho$):

- [IEEET1](Model/PhasorDynamics/Exciter/IEEET1/README.md): smooth magnetic saturation above the saturation knee
- [REGCA](Model/PhasorDynamics/Converter/REGCA/README.md): applies reactive-current and active-current rate-limit corrections

Binary min/max (`Math::min`, `Math::max`):

- [REECA](Model/PhasorDynamics/Converter/REECA/README.md): forms the safe measured voltage floor and final current limits

Linear segment (`Math::linseg`, $\text{linseg}$):

- [REGCA](Model/PhasorDynamics/Converter/REGCA/README.md): defines the LVPL and LVACM piecewise-linear curves
- [REECA](Model/PhasorDynamics/Converter/REECA/README.md): defines smooth voltage-dependent current-limit interpolation curves

Deadband (`Math::deadband`):

- [REECA](Model/PhasorDynamics/Converter/REECA/README.md): smooths the two-sided voltage-error deadband

Inside indicator (`Math::inside`):

- [IEEEST](Model/PhasorDynamics/Stabilizer/IEEEST/README.md): forms the smooth output limiter for $v_7$ over $[L_{smin}, L_{smax}]$

Outside indicator (`Math::outside`):

- [REECA](Model/PhasorDynamics/Converter/REECA/README.md): forms the smooth voltage dip/overvoltage indicator for $s_{\mathrm{dip}}$

Anti-windup (`Math::antiwindup`, with gate `Math::indicator`):

- [IEEET1](Model/PhasorDynamics/Exciter/IEEET1/README.md): gates $\dot V_R$ on $V_R \in (V_{rmin}, V_{rmax})$
- [TGOV1](Model/PhasorDynamics/Governor/Tgov1/README.md): gates $\dot P_v$ on $P_v \in (P_{vmin}, P_{vmax})$
- [SEXS-PTI](Model/PhasorDynamics/Exciter/SEXS-PTI/README.md): gates $\dot E_{fd}$ on $E_{fd} \in (E_{fd,\min}, E_{fd,\max})$
- [REECA](Model/PhasorDynamics/Converter/REECA/README.md): gates PI-controller and active-power-order dynamics during output saturation

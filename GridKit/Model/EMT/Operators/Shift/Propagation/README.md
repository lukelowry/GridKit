# Propagation Model

`Propagation` represents the current-form EMT propagation operator used by
`LineDistributed`. It composes two fitted `StateSpace` factors with one scalar
`Delay` block per propagation mode.

```math
\mathbf{H}_i(s)
=
\mathbf{F}_{\mathrm{out}}(s)
\boldsymbol{\Delta}_{\tau}(s)
\mathbf{F}_{\mathrm{in}}(s)
```

where

```math
\begin{aligned}
\mathbf{F}_{\mathrm{in}}(s) &\approx
\widehat{\mathbf{H}}^{\mathrm{mps}}(s)\mathbf{T}_v(s)^{\mathsf H} \\
\boldsymbol{\Delta}_{\tau}(s) &=
\operatorname{diag}\left(e^{-s\tau_1},\ldots,e^{-s\tau_M}\right) \\
\mathbf{F}_{\mathrm{out}}(s) &\approx
\mathbf{T}_i(s).
\end{aligned}
```

The state-space factors use the existing `StateSpace` form:

```math
\mathbf{F}_{r}(s)
\approx
\mathbf{D}_{r}
+ s\mathbf{E}_{r}
+ \mathbf{C}_{r}(s\mathbf{I}-\mathbf{P}_{r})^{-1}\mathbf{B}_{r},
\qquad
r \in \{\mathrm{in},\mathrm{out}\}.
```

The Laplace domain representation of this model is:

```math
\mathbf{Y}(s)=\mathbf{H}_i(s)\mathbf{U}(s)
```

The time domain representation of this model is:

```math
\mathbf{y}(t)=(\mathbf{h}_i*\mathbf{u})(t)
```

## Block Diagram

<div align="center">
   <img align="center" src="../../../../../../docs/Figures/EMT/Propagation/diagram.png">

  Figure 1: Propagation model
</div>

## Model Parameters

For conductor count $K$ and modal count $M$:

Symbol | Units | JSON | Description | Note
------ | ----- | ---- | ----------- | ----
$\mathbf{F}_{\mathrm{in}}$ | [-] | `input` | Input-side fitted factor | `StateSpace`, $M \times K$
$\boldsymbol{\tau}$ | [s] | `tau` | Modal propagation delays | $\boldsymbol{\tau}\in\mathbb{R}^M$
$\Delta t_{\min}$ | [s] | `dt_min` | Delay block resolution | passed to each scalar `Delay`
$\mathbf{F}_{\mathrm{out}}$ | [-] | `output` | Output-side fitted factor | `StateSpace`, $K \times M$

### Parameter Validation

```math
\begin{aligned}
\tau_m &> 0,\qquad m=1,\ldots,M \\
\Delta t_{\min} &> 0
\end{aligned}
```

The `StateSpace` child models validate their own poles and factor matrices.

### Model Derived Parameters

None. The child `StateSpace` and `Delay` models own their derived parameters.

### Model Submodels

Submodel | Inputs | Outputs
-------- | ------ | -------
[`StateSpace`](../../Rational/StateSpace/README.md) $\mathbf{F}_{\mathrm{in}}$ | $\mathbf{u}\in\mathbb{R}^K$ | $\hat{\mathbf{u}}\in\mathbb{R}^M$
[`Delay`](../Delay/README.md) $\delta_m$, $m=1,\ldots,M$ | $\hat{u}_m$, $\tau_m$ | $\hat{z}_m$
[`StateSpace`](../../Rational/StateSpace/README.md) $\mathbf{F}_{\mathrm{out}}$ | $\hat{\mathbf{z}}\in\mathbb{R}^M$ | $\mathbf{y}\in\mathbb{R}^K$

## Model Variables

### Internal Variables

#### Differential

None. The child `StateSpace` and `Delay` models own their differential states.

#### Algebraic

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\hat{\mathbf{u}}$ | [-] | Modal signal after input-side factor | $\hat{\mathbf{u}}\in\mathbb{R}^M$
$\hat{\mathbf{z}}$ | [-] | Delayed modal signal | $\hat{\mathbf{z}}\in\mathbb{R}^M$

### External Variables

#### Differential

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{u}$ | [-] | Input vector | $\mathbf{u}\in\mathbb{R}^K$

#### Algebraic

None.

## Model Ports

Symbol | Port | Type | Units | Description | Note
------ | ---- | ---- | ----- | ----------- | ----
$\mathbf{u}$ | `input` | Input | [-] | Input vector port | $\mathbf{u}\in\mathbb{R}^K$
$\mathbf{y}$ | `out` | Output | [-] | Output contribution port | $\mathbf{y}\in\mathbb{R}^K$

## Model Equations

### Differential Equations

None. The child `StateSpace` and `Delay` models own their differential equations.

### Algebraic Equations

None.

### Port Equations

```math
\begin{aligned}
\hat{\mathbf{u}} &= \mathbf{f}_{\mathrm{in}} * \mathbf{u} \\
\hat{z}_m &= \delta(t-\tau_m) * \hat{u}_m,\qquad m=1,\ldots,M \\
\mathbf{y} &= \mathbf{f}_{\mathrm{out}} * \hat{\mathbf{z}}.
\end{aligned}
```

## Initialization

Initialization is delegated to the child `StateSpace` and `Delay` models. The
wrapper initializes internal signals with the same interconnection equations:

```math
\begin{aligned}
\hat{\mathbf{u}}_0 &= (\mathbf{f}_{\mathrm{in}} * \mathbf{u})_0 \\
\hat{z}_{m,0} &= \hat{u}_{m,0},\qquad m=1,\ldots,M \\
\mathbf{y}_0 &= (\mathbf{f}_{\mathrm{out}} * \hat{\mathbf{z}})_0.
\end{aligned}
```

## Monitors

None.

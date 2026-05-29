# **Hydro Turbine-Governor Model (HYGOV)**

HYGOV is a hydro turbine-governor model with temporary droop, a gate servo, and
a nonlinear single-penstock turbine. It reads machine speed deviation and
outputs mechanical power.

Notes:
- PowerWorld uses the connected machine base when `Trate = 0`; GridKit requires
  `Trate` to be set explicitly.
- HYGOVD `dbL`/`dbH`, `db2` backlash, and Kaplan blade-servo fields are not
  modeled.

## Block Diagram

Standard HYGOV block diagram.

![](../../../../../docs/Figures/PhasorDynamics/HYGOV_diagram.png)

Figure 1: Governor HYGOV model. Figure courtesy of [PowerWorld](https://www.powerworld.com/WebHelp/)

## Model Parameters

Symbol                          | Units    | JSON     | Description                                  | Typical Value | Note
--------------------------------|----------|----------|----------------------------------------------|---------------|------
$T_{\mathrm{rate}}$             | [MW]     | `Trate` | Component-base turbine rating                | 100.0         | Required positive value
$R_{\mathrm{perm}}$             | [p.u.]   | `Rperm`  | Permanent droop                              | 0.04          | Source diagram label: `R`
$R_{\mathrm{temp}}$             | [p.u.]   | `Rtemp`  | Temporary droop                              | 0.3           | Source diagram label: `r`
$T_r$                           | [sec]    | `Tr`     | Temporary-droop reset time constant          | 5.0           |
$T_f$                           | [sec]    | `Tf`     | Governor error filter time constant          | 0.05          | State 1
$T_g$                           | [sec]    | `Tg`     | Gate servo time constant                     | 0.5           | State 3
$V_{\mathrm{elm}}$              | [p.u./s] | `Velm`   | Maximum desired-gate velocity magnitude      | 0.2           | Symmetric rate limit on State 2
$G^{\max}$                      | [p.u.]   | `Gmax`   | Maximum desired-gate position                | 1.0           |
$G^{\min}$                      | [p.u.]   | `Gmin`   | Minimum desired-gate position                | 0.0           |
$T_w$                           | [sec]    | `Tw`     | Water inertia time constant                  | 1.0           | State 4
$A_t$                           | [p.u.]   | `At`     | Turbine gain                                 | 1.2           |
$D_{\mathrm{turb}}$             | [p.u.]   | `Dturb`  | Turbine damping coefficient                  | 0.5           | Multiplied by speed deviation and gate
$q_{\mathrm{NL}}$               | [p.u.]   | `Qnl`    | No-load flow at nominal head                 | 0.05          |
$T_n$                           | [sec]    | `Tn`     | Speed lead-lag numerator time constant       | 0.0           |
$T_{np}$                        | [sec]    | `Tnp`    | Speed lead-lag denominator time constant     | 0.0           |
$D_{\omega}$                    | [p.u.]   | `db1`    | Type 1 speed deadband threshold              | 0.0           | Uses CommonMath `deadband1`
$H_{\mathrm{dam}}$              | [p.u.]   | `Hdam`   | Head available at dam                        | 1.0           |
$G_V^{(k)}$                     | [p.u.] | `Gv0`-`Gv5`   | Gate point $k$ of the gain curve             | 0.0           | $k=0,\ldots,5$
$P_{\mathrm{GV}}^{(k)}$         | [p.u.] | `Pgv0`-`Pgv5` | Power point $k$ of the gain curve            | 0.0           | $k=0,\ldots,5$

All-zero `Gv` and `Pgv` source points select the identity curve.

### Parameter Validation

Invalid HYGOV parameter sets are rejected by the following checks. The displayed
equations use effective time constants with $\epsilon_T=10^{-3}$.

```math
\begin{aligned}
  T &\leftarrow \max\!\left(T, \epsilon_T\right)
    \quad T\in\{T_r,T_f,T_g,T_w,T_{np}\} \\
  T_{\mathrm{rate}}, H_{\mathrm{dam}}
    &> 0 \\
  R_{\mathrm{temp}}
    &\ne 0 \\
  V_{\mathrm{elm}}, D_{\omega}
    &\ge 0 \\
  G^{\min} &\le G^{\max} \\
  G_V^{(k)} &< G_V^{(k+1)}
    \quad k\in\{0,\ldots,4\} \\
  P_{\mathrm{GV}}^{(k)} &\le P_{\mathrm{GV}}^{(k+1)}
    \quad k\in\{0,\ldots,4\}
\end{aligned}
```

Initialization also requires $N_{\mathrm{GV}}^{-1}$ to be single-valued at the
initial operating point.

### Model Derived Parameters

The speed lead-lag coefficient and nonlinear gate-to-power curve are:

```math
\begin{aligned}
  k_n &=
    \dfrac{T_n}{T_{np}} \\
  N_{\mathrm{GV}}(x)
    &=
      P_{\mathrm{GV}}^{(0)}
      + \sum_{k\in\{0,\ldots,4\}}
        \text{linseg}\!\left(
          x;\,
          G_V^{(k)},\,
          G_V^{(k+1)},\,
          P_{\mathrm{GV}}^{(k+1)} - P_{\mathrm{GV}}^{(k)}
        \right)
\end{aligned}
```

CommonMath defines the [linear segment](../../../../CommonMath.md#derived-functions)
helper used by $N_{\mathrm{GV}}$.

## Model Variables

### Internal Variables

#### Differential

Symbol                  | Units  | Description                         | Note
------------------------|--------|-------------------------------------|------
$x_n$                   | [p.u.] | Speed lead-lag denominator state    | Not circled in Fig. 1; realizes the `Tn`/`Tnp` block
$x_f$                   | [p.u.] | Governor error filter output        | State 1 in Fig. 1
$c$                     | [p.u.] | Desired-gate position               | State 2 in Fig. 1
$g$                     | [p.u.] | Gate position                       | State 3 in Fig. 1
$q$                     | [p.u.] | Turbine flow                        | State 4 in Fig. 1

#### Algebraic

Symbol                            | Units    | Description                         | Note
----------------------------------|----------|-------------------------------------|------
$\omega_{\mathrm{db}}$            | [p.u.]   | Type 1 deadbanded speed deviation   | Defined by CommonMath `deadband1`
$e_f$                             | [p.u.]   | Governor error into the filter      | Reference path less conditioned speed and permanent-droop feedback
$f_c$                             | [p.u./s] | Desired-gate derivative target      | Before rate and position limits
$r_c$                             | [p.u./s] | Rate-limited desired-gate derivative target | Limited by $\pm V_{\mathrm{elm}}$
$P_{\mathrm{GV}}$                 | [p.u.]   | Nonlinear gate-to-power curve output | $N_{\mathrm{GV}}(g)$
$H$                               | [p.u.]   | Turbine head                        | Implicit water-column head
$P_{\text{m}}$                    | [p.u.]   | Mechanical power to generator       | Read by a machine model

### External Variables

#### Differential
None.

#### Algebraic

Symbol                          | Units  | Description                    | Note
--------------------------------|--------|--------------------------------|------
$\omega$                        | [p.u.] | Machine speed deviation        | Read from a machine model
$P_{\mathrm{ref}}$              | [p.u.] | Active-power/load reference    | HYGOV component base; external setpoint or constant parameter; source label: `Pref`
$P_{\mathrm{aux}}$              | [p.u.] | Auxiliary power input          | HYGOV component base; optional, defaults to zero; source label: `Paux`

## Model Equations

### Differential Equations

```math
\begin{aligned}
  0 &=
    -\dot{x}_n
    + \dfrac{1}{T_{np}}
      \left(\omega_{\mathrm{db}} - x_n\right) \\
  0 &=
    -\dot{x}_f
    + \dfrac{1}{T_f}
      \left(e_f - x_f\right) \\
  0 &=
    -\dot{c}
    + \text{antiwindup}
      \left(c, r_c;\, G^{\min}, G^{\max}\right) \\
  0 &=
    -\dot{g}
    + \dfrac{1}{T_g}
      \left(c - g\right) \\
  0 &=
    -\dot{q}
    + \dfrac{1}{T_w}
      \left(H_{\mathrm{dam}} - H\right)
\end{aligned}
```

CommonMath defines the [Anti-Windup](../../../../CommonMath.md#antiwindup)
helper.

### Algebraic Equations

```math
\begin{aligned}
  0 &=
    -\omega_{\mathrm{db}}
    + \text{deadband1}
      \left(\omega;\, -D_{\omega}, D_{\omega}\right) \\
  0 &=
    -e_f
    + P_{\mathrm{ref}}
    + P_{\mathrm{aux}}
    - x_n
    - k_n\left(\omega_{\mathrm{db}} - x_n\right)
    - R_{\mathrm{perm}}c \\
  0 &=
    -f_c
    + \dfrac{1}{R_{\mathrm{temp}}}
      \left[
        \dfrac{x_f}{T_r}
        + \dfrac{e_f - x_f}{T_f}
      \right] \\
  0 &=
    -r_c
    + \text{clamp}
      \left(f_c;\, -V_{\mathrm{elm}}, V_{\mathrm{elm}}\right) \\
  0 &=
    -P_{\mathrm{GV}}
    + N_{\mathrm{GV}}(g) \\
  0 &=
    -q^2
    + H P_{\mathrm{GV}}^2 \\
  0 &=
    -\dfrac{S_{\mathrm{sys}}}{T_{\mathrm{rate}}}P_{\text{m}}
    + \left[
        A_t H\left(q - q_{\mathrm{NL}}\right)
        - D_{\mathrm{turb}}\omega g
      \right]
\end{aligned}
```

CommonMath defines helper targets and smooth approximations for
[deadband1 and clamp](../../../../CommonMath.md#derived-functions).

## Initialization

Initialization is performed by evaluating the steady-state residuals in
dependency order. Let subscript $0$ denote initial values and set all internal
derivatives to zero:

```math
\begin{aligned}
  \omega_0 &= 0 \\
  P_{\mathrm{aux},0} &= 0 \\
  H_0 &= H_{\mathrm{dam}} \\
  q_0
    &= q_{\mathrm{NL}}
       + \dfrac{S_{\mathrm{sys}}P_{\text{m},0}}
               {T_{\mathrm{rate}}A_tH_0} \\
  P_{\mathrm{GV},0} &= \dfrac{q_0}{\sqrt{H_0}} \\
  g_0 &= N_{\mathrm{GV}}^{-1}\!\left(P_{\mathrm{GV},0}\right) \\
  c_0 &= g_0 \\
  \omega_{\mathrm{db},0}
    &= \text{deadband1}\!\left(\omega_0, -D_{\omega}, D_{\omega}\right) \\
  x_{n,0} &= \omega_{\mathrm{db},0} \\
  x_{f,0} &= 0 \\
  e_{f,0} &= x_{f,0} \\
  f_{c,0} &= 0 \\
  r_{c,0} &= 0 \\
  P_{\mathrm{ref},0}
    &= e_{f,0}
       - P_{\mathrm{aux},0}
       + x_{n,0}
       + k_n\left(\omega_{\mathrm{db},0} - x_{n,0}\right)
       + R_{\mathrm{perm}}c_0
\end{aligned}
```

This closed-form start requires $G^{\min} \le c_0 \le G^{\max}$ and a
single-valued inverse $N_{\mathrm{GV}}^{-1}(P_{\mathrm{GV},0})$.

## Monitorable Output

Output         | Units  | Description                         | Note
---------------|--------|-------------------------------------|------
`pmech`        | [p.u.] | Mechanical-power output             | $P_{\text{m}}$
`filter`       | [p.u.] | Governor error filter output        | State 1
`desiredgate`  | [p.u.] | Desired-gate position               | State 2
`gate`         | [p.u.] | Gate position                       | State 3
`flow`         | [p.u.] | Turbine flow                        | State 4
`head`         | [p.u.] | Turbine head                        | Implicit water-column head

# **IEEE Stabilizer Model (IEEEST)**

Standard IEEE power system stabilizer: up-to-fourth-order notch filter, two
lead–lag blocks, washout, and output limiter.

## Notes

- $V_{\mathrm{cl}}$, $V_{\mathrm{cu}}$, and $T_{\mathrm{delay}}$ are accepted for input-format
  compatibility but are not modeled.

## Block Diagram

![](../../../../../docs/Figures/stabilizer_ieeest_diagram.png)

Figure 1: Stabilizer IEEEST model. Figure courtesy of [PowerWorld](https://www.powerworld.com/WebHelp/)

## Model Parameters

Symbol                  | Units    | JSON     | Description                          | Typical Value | Note
------------------------|----------|----------|--------------------------------------|---------------|------
$A_1$                   | [sec]    | `A1`     | Notch denominator coefficient        | 1.013         |
$A_2$                   | [sec²]   | `A2`     | Notch denominator coefficient        | 0.013         |
$A_3$                   | [sec]    | `A3`     | Notch denominator coefficient        | 0.0           |
$A_4$                   | [sec²]   | `A4`     | Notch denominator coefficient        | 0.0           |
$A_5$                   | [sec]    | `A5`     | Notch numerator coefficient          | 1.013         |
$A_6$                   | [sec²]   | `A6`     | Notch numerator coefficient          | 0.113         |
$T_1$                   | [sec]    | `T1`     | Lead–lag 1 numerator time constant   | 0.0           |
$T_2$                   | [sec]    | `T2`     | Lead–lag 1 denominator time constant | 0.02          |
$T_3$                   | [sec]    | `T3`     | Lead–lag 2 numerator time constant   | 0.0           |
$T_4$                   | [sec]    | `T4`     | Lead–lag 2 denominator time constant | 0.0           |
$T_5$                   | [sec]    | `T5`     | Washout numerator time constant      | 1.65          |
$T_6$                   | [sec]    | `T6`     | Washout denominator time constant    | 1.65          |
$K_s$                   | [p.u.]   | `Ks`     | Stabilizer gain                      | 3.0           |
$L_s^{\min}$      | [p.u.]   | `Lsmin`  | Minimum stabilizer output limit      | -0.1          |
$L_s^{\max}$      | [p.u.]   | `Lsmax`  | Maximum stabilizer output limit      | 0.1           |
$V_{\mathrm{cl}}$       | [p.u.]   | `Vcl`    | Lower input cutout threshold         | 0.0           | Accepted but not modeled
$V_{\mathrm{cu}}$       | [p.u.]   | `Vcu`    | Upper input cutout threshold         | 0.0           | Accepted but not modeled
$T_{\mathrm{delay}}$    | [sec]    | `Tdelay` | Input delay                          | 0.0           | Accepted but not modeled

### Parameter Validation

IEEEST denominator time constants are conditioned and unsupported notch forms are rejected by the following checks. Let $\epsilon_T=10^{-3}$.

```math
\begin{aligned}
  T &\leftarrow \max\!\left(T, \epsilon_T\right)
    \quad T\in\{T_2,T_4,T_6\} \\
  A_5 &= A_6 = 0
    \quad\text{when}\quad
    n = 0 \\
  A_6 &= 0
    \quad\text{when}\quad
    n = 1
\end{aligned}
```

### Model Derived Parameters

```math
\begin{aligned}
  a_1 &= A_1 + A_3 \\
  a_2 &= A_2 + A_4 + A_1 A_3 \\
  a_3 &= A_1 A_4 + A_2 A_3 \\
  a_4 &= A_2 A_4 \\
  n &=
    \begin{cases}
      4 & a_4 \ne 0 \\
      3 & a_4 = 0,\ a_3 \ne 0 \\
      2 & a_4 = a_3 = 0,\ a_2 \ne 0 \\
      1 & a_4 = a_3 = a_2 = 0,\ a_1 \ne 0 \\
      0 & a_4 = a_3 = a_2 = a_1 = 0
    \end{cases}
\end{aligned}
```

## Model Ports

Name     | Port   | Init  | Description
---------|--------|-------|------
`input`  | Input  | Known | Stabilizer input signal
`output` | Output | Known | Stabilizer output signal

## Model Variables

### Internal Variables

#### Differential

Symbol                | Units  | Description           | Note
----------------------|--------|-----------------------|------
$x_1, x_2, x_3, x_4$  | [-]    | Notch filter states   | States 1–4 in Fig. 1
$x_5$                 | [-]    | Lead–lag 1 state      | State 5 in Fig. 1
$x_6$                 | [-]    | Lead–lag 2 state      | State 6 in Fig. 1
$x_7$                 | [-]    | Washout state         | State 7 in Fig. 1

#### Algebraic

Symbol               | Units  | Description                              | Note
---------------------|--------|------------------------------------------|------
$v_4$                | [p.u.] | Notch filter output                      |
$v_5$                | [p.u.] | Lead–lag 1 output                        |
$v_6$                | [p.u.] | Lead–lag 2 output                        |
$v_7$                | [p.u.] | Unlimited stabilizer signal              |
$V_{\mathrm{ss}}$    | [p.u.] | Limited stabilizer signal (model output) |

### External Variables

#### Differential

None.

#### Algebraic

Symbol | Units  | Type  | Description             | Note
-------|--------|-------|-------------------------|------
$u$    | [p.u.] | Known | Stabilizer input signal |

## Model Equations

### Differential Equations

```math
\begin{aligned}
  0 &=
    -\dot{x}_1
    + \begin{cases}
      0, & n = 0 \\
      \dfrac{1}{a_1}\left(-x_1 + u\right), & n = 1 \\
      x_2, & n = 2,3,4
    \end{cases} \\
  0 &=
    -\dot{x}_2
    + \begin{cases}
      0,
        & n = 0,1 \\
      \dfrac{1}{a_2}\left(-x_1 - a_1x_2 + u\right),
        & n = 2 \\
      x_3,
        & n = 3,4
    \end{cases} \\
  0 &=
    -\dot{x}_3
    + \begin{cases}
      0,
        & n = 0,1,2 \\
      \dfrac{1}{a_3}\left(-x_1 - a_1x_2 - a_2x_3 + u\right),
        & n = 3 \\
      x_4,
        & n = 4
    \end{cases} \\
  0 &=
    -\dot{x}_4
    + \begin{cases}
      0,
        & n = 0,1,2,3 \\
      \dfrac{1}{a_4}\left(-x_1 - a_1x_2 - a_2x_3 - a_3x_4 + u\right),
        & n = 4
    \end{cases} \\
  0 &=
    -\dot{x}_5
    + \dfrac{1}{T_2}\left(v_4 - x_5\right) \\
  0 &=
    -\dot{x}_6
    + \dfrac{1}{T_4}\left(v_5 - x_6\right) \\
  0 &=
    -\dot{x}_7
    + \dfrac{1}{T_6}\left(v_6 - x_7\right)
\end{aligned}
```

### Algebraic Equations

```math
\begin{aligned}
  0 &=
    -v_4
    + \begin{cases}
      u,
        & n = 0 \\
      x_1 + \dfrac{A_5}{a_1}\left(-x_1 + u\right),
        & n = 1 \\
      x_1 + A_5x_2
        + \dfrac{A_6}{a_2}\left(-x_1 - a_1x_2 + u\right),
        & n = 2 \\
      x_1 + A_5x_2 + A_6x_3,
        & n = 3,4
    \end{cases} \\
  0 &= -v_5 + x_5 + \dfrac{T_1}{T_2}\left(v_4 - x_5\right) \\
  0 &= -v_6 + x_6 + \dfrac{T_3}{T_4}\left(v_5 - x_6\right) \\
  0 &= -v_7 + K_s\dfrac{T_5}{T_6}\left(v_6 - x_7\right) \\
  0 &=
    -V_{\mathrm{ss}}
    + \text{clamp}(v_7, L_s^{\min}, L_s^{\max})
\end{aligned}
```

The output limiter uses GridKit's smooth
[clamp](../../../../CommonMath.md#derived-functions).

## Initialization

### Input Initialization

```math
\begin{aligned}
  u &\leftarrow \text{stabilizer input signal}
\end{aligned}
```

### Internal Initialization

```math
\begin{aligned}
  x_{1,0} &= v_{4,0} = x_{5,0} = v_{5,0} = x_{6,0} = v_{6,0} = x_{7,0} = u_0 \\
  x_{2,0} &= x_{3,0} = x_{4,0} = 0 \\
  v_{7,0} &= 0 \\
  V_{\mathrm{ss},0} &= \text{clamp}(v_{7,0}, L_s^{\min}, L_s^{\max}) \\
  \dot{x}_{i,0} &= 0 \quad i\in\{1,\ldots,7\}
\end{aligned}
```

### Output Initialization

None.

## Monitorable Outputs

Output | Units  | Description               | Note
-------|--------|---------------------------|------
`vss`  | [p.u.] | Limited stabilizer signal | Exported through `output` when assigned

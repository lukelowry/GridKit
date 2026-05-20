# LoadRL Model

`LoadRL` represents a three-phase RL load in instantaneous abc coordinates.
The load owns the three-phase differential current vector $\mathbf{i}$,
which is directed from the load into the bus.

## Model Parameters

Symbol  | Units      | Description                | Note
--------|------------|----------------------------|--------------------------------
$R_a$   | [$\Omega$] | Load resistance, phase a    |
$R_b$   | [$\Omega$] | Load resistance, phase b    |
$R_c$   | [$\Omega$] | Load resistance, phase c    |
$L_a$   | [H]        | Load inductance, phase a    |
$L_b$   | [H]        | Load inductance, phase b    |
$L_c$   | [H]        | Load inductance, phase c    |

## Model Derived Parameters

``` math
\begin{aligned}
  \mathbf{R} &= \operatorname{diag}(R_a, R_b, R_c) \\
  \mathbf{L} &= \operatorname{diag}(L_a, L_b, L_c)
\end{aligned}
```

## Model Variables

### Internal Variables

#### Differential

Symbol              | Units | Description                                    | Note
--------------------|-------|------------------------------------------------|---------------------------------
$\mathbf{i}$        | [A]   | Load current vector, directed from load into bus | $\mathbf{i} = [i_a, i_b, i_c]^T \in \mathbb{R}^3$

#### Algebraic

None.

### External Variables

External variables enter component model equations but are owned by
other components. The EMT bus at the load port owns the voltage
variable and provides the equation needed to have a balanced system
of equations.

#### Differential

Symbol           | Units | Description                                  | Note
-----------------|-------|----------------------------------------------|---------------------------------
$\mathbf{v}$     | [V]   | Port voltage vector, owned by EMT bus        | $\mathbf{v} = [v_a, v_b, v_c]^T \in \mathbb{R}^3$

#### Algebraic

None.

## Model Equations

### Differential Equations

``` math
0 = \mathbf{R}\,\mathbf{i} + \mathbf{L}\dot{\mathbf{i}} + \mathbf{v}
```

### Algebraic Equations

None.

### Bus Residual Contributions

The RL load contributes to the KCL residual at its port bus. The
expression is accumulated into the owning bus residual.

``` math
\mathbf{i}^\text{inj} := \mathbf{i}
```

## Initialization

The initialization uses the physical load parameters and the connected bus
initial RMS phase-voltage phasors. For each phase, with
$V$ denoting the RMS bus phasor and $\omega$ denoting the bus angular
frequency,

``` math
Z = R + j\omega L
```

``` math
I = -\frac{V}{Z}
```

``` math
i(0) = \sqrt{2}\,\Re(I)
```

``` math
\dot{i}(0) = \sqrt{2}\,\Re(j\omega I)
```

## Model Outputs

Candidate monitorable outputs include the load current components $i_a$, $i_b$,
and $i_c$ into the bus.

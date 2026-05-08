# SignalDelay

The **SignalDelay** model consumes one existing `SignalNode` and publishes one
delayed `SignalNode`. It is implemented as a normal PhasorDynamics component and
does not wrap or replace `SignalNode`.

The inherited GridKit state vector is named `y_` by convention. In this model,
the delayed output signal is denoted by $v_d$ and is stored in `y_[0]`.

## Model Parameters

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\tau$ | [s] | Fixed signal delay | `delay`, nonnegative
$v_{\mathrm{init}}$ | [-] | Value before history exists | `initial_value`
mode | [-] | Accepted-step history lookup mode | `SignalDelayMode::LINEAR` by default
max_step_size | [s] | Maximum accepted solver step | Nonpositive values use $\tau$

The lookup modes are:

- `SignalDelayMode::LINEAR`: piecewise-linear interpolation between accepted
  solver-step samples. This is the default mode and is intended for smooth
  waveform and current propagation.
- `SignalDelayMode::HOLD`: zero-order hold of the last accepted solver-step
  sample at or before $t - \tau$. This preserves sample-held
  or intentionally discontinuous semantics.

## Model Variables

### Internal Variables

#### Differential

None.

#### Algebraic

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$v_d$ | [-] | Delayed output signal value | Stored in `y_[0]`

### External Variables

#### Differential

None.

#### Algebraic

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$u$ | [-] | Input signal value | Read from attached `SignalNode`

## Model Equations

### Differential Equations

None.

### Algebraic Equations

For positive delay, the component evaluates the selected accepted-step-history
lookup operator:

```math
0 = v_d - \mathcal{D}_{\tau}(u)(t)
```

For zero delay, the component reads the current input signal directly:

```math
0 = v_d - u
```

This avoids adding a one-step lag when the configured delay is zero.

## Initialization

The delay history is empty at initialization. The output is initialized to the
configured initial value for positive delay:

```math
v_{d0} = v_{\mathrm{init}}
```

For zero delay, the output is initialized from the current input signal:

```math
v_{d0} = u(t_0)
```

The state is algebraic, so its stored derivative is initialized to zero.

## Notes

- `SignalDelay` stores accepted-step input samples through `History`.
- After each accepted step is recorded, samples older than the active delay
  window are removed.
- One scalar anchor sample is retained immediately before the active delay
  window. The anchor is needed for hold lookup and for linear interpolation when
  no accepted-step sample lands exactly at $t - \tau$.
- Retained memory is proportional to the number of accepted-step samples inside
  the delay window, plus one anchor sample. It does not grow with total
  simulation time.

## Numerical Meaning

`SignalDelay` is a DAE component with accepted-step history lookup. It is not DDE
support.

IDA residual and Jacobian calls may evaluate trial states that are later
rejected. For that reason, `SignalDelay` does not update history in residual or
Jacobian evaluation. History is updated only through `stepAccepted`, after the
GridKit IDA driver receives an accepted returned solution from `IDASolve` or
`IDASolveF` and copies that state back into the model.

For positive delay, `SignalDelay` requests a maximum accepted solver step. The
default maximum is $\tau$; `max_step_size > 0` requests
`min(delay, max_step_size)`. This keeps the accepted-step history spacing small
enough that delayed lookups do not require future, unrecorded input samples.

For discontinuous inputs, exact delayed jump timing depends on accepted-step
solution times. If a jump at $t_j$ must appear exactly at $t_j + \tau$, the
simulation should include output or future breakpoint support at those
times.

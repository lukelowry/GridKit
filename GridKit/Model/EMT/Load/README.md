# EMT Shunt Load Model

The **ShuntLoad** model represents a three-phase resistive shunt load in
instantaneous abc coordinates.

## Model Parameters

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{G}$ | [p.u.] | Shunt conductance matrix | Row-major `3 x 3`
$s_0$ | [bool] | Initial switch state | `ShuntLoadData::closed`

## Model Variables

### Internal Variables

#### Differential

None.

#### Algebraic

None. Terminal current is a computed output.

### External Variables

#### Differential

None.

#### Algebraic

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{v}$ | [p.u.] | Connected bus voltage | Owned by the connected bus

## Model Equations

### Differential Equations

None.

### Algebraic Equations

The load current is

```math
\mathbf{i}_\ell(t) = s \mathbf{G}\mathbf{v}(t)
```

where $s = 1$ when the load is closed and $s = 0$ when it is open.

The current is oriented from the bus into the load, so its bus current-balance
contribution is $-\mathbf{i}_\ell$.

## Initialization

No internal variables are initialized.

## Model Outputs

Symbol | Units | Description | Note
------ | ----- | ----------- | ----
$\mathbf{i}_\ell$ | [p.u.] | Terminal load current | Positive from bus into load

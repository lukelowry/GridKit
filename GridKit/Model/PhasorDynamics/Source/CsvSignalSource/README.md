# CsvSignalSource

The **CsvSignalSource** model publishes one algebraic signal from tabular CSV
data. It is intended for simple forcing functions used by other phasor dynamics
components.

The inherited GridKit state vector is named `y_` by convention. In this model,
the published signal value is denoted by $v$ and is stored in `y_[0]`.

## Model Parameters

Field | Default | Description
----- | ------- | -----------
`file` | empty | CSV file containing input samples
`time_column` | `t` | Name of the time column
`value_column` | `u` | Name of the signal value column
`value_scale` | `1.0` | Multiplicative value scale
`value_offset` | `0.0` | Additive value offset after scaling

## Model Equations

The source enforces the published value against the CSV-defined input function:

```math
0 = v - u_{\mathrm{csv}}(t)
```

The function $u_{\mathrm{csv}}(t)$ is evaluated with piecewise-linear
interpolation between CSV samples.

## Initialization

The initial published value is computed from the CSV function at the current
component time:

```math
v_0 = u_{\mathrm{csv}}(t_0)
```

The state is algebraic, so its stored derivative is initialized to zero.

## CSV Format

The CSV must contain a header row and at least two data rows. Time samples must
be strictly increasing.

```text
t,u
0.0,1.0
0.1,0.9800665778412416
```

Quoted fields and escaped delimiters are not supported.

## Notes

- Values are scaled as `value_scale * raw_value + value_offset`.
- The interpolation mode is fixed to piecewise linear in this component.

# Input file for GridKit phasor dynamics application

## Root elements

   Name               | Value
 ---------------------|-------------------------------------------------------
  `system_model_file` | Path to the system model file[^1]
  `dt_monitor`        | Monitor output time interval for recorded simulation results (default: 0, no intermediate monitoring)[^2]
  `tmax`              | A floating-point value for max time
  `ida`               | IDA solver options (optional; see [IDA options](#ida-options))
  `fault_bus`         | Bus where the study's bus fault is applied (optional; defaults to the bus in the system model)
  `events`            | An array of event groups (see [Events](#events) below)
  `output_file`       | Path to output (CSV) file (optional)
  `reference_file`    | A string containing the name of the case (optional)
  `error_type`        | One of { "relative" (default), "absolute" }
  `error_tolerance`   | A floating-point value for highest allowable total error (default: 1.0e-4)
  `abs_err_threshold` | A floating-point value for the smallest value at which to scale relative error (default: machine epsilon for double-precision)

[^1]: See system model [case format](../../GridKit/Model/PhasorDynamics/INPUT_FORMAT.md)

[^2]: Accepted under the deprecated name `dt` for backward compatibility. `dt_monitor` takes precedence when both are given.

## IDA options

All IDA options are optional. Omitted options use the listed default.

 Name                        | Default
 ----------------------------|-----------------------
 `rel_tol`                   | `1.0e-7`
 `abs_tol`                   | `1.0e-9`; use `0` for model-specific tolerances
 `fixed_step`                | Adaptive stepping
 `init_step`                 | Estimated by IDA
 `min_step`                  | No minimum
 `max_step`                  | Unbounded
 `max_order`                 | `5`
 `max_num_steps`             | `500`
 `max_err_test_fails`        | `10`
 `suppress_alg`              | `false`
 `max_nonlin_iters`          | `4`
 `max_conv_fails`            | `10`
 `nonlin_conv_coef`          | `0.33`
 `max_num_steps_ic`          | `5`
 `max_num_jacs_ic`           | `4`
 `max_num_iters_ic`          | `10`
 `max_backs_ic`              | `100`
 `line_search_off_ic`        | `false`
 `nonlin_conv_coef_ic`       | `0.0033`
 `step_tolerance_ic`         | IDA default
 `linear_solution_scaling`   | `true`
 `delta_cj_lsetup`           | `0.25`
 `klu_ordering`              | `"amd"`; one of `"amd"`, `"colamd"`, or `"natural"`

`fixed_step` sets both the integration mode and step size. It cannot be combined
with `init_step`, `min_step`, or `max_step`.

`klu_ordering` selects the fill-reducing ordering KLU applies to the Jacobian.
`"amd"` suits the near-symmetric network matrices these models produce and is
the default; the other orderings are provided for comparison.

```json
"ida": {
    "rel_tol": 1.0e-6,
    "max_num_steps": 2000
}
```

## Events

Each event group describes a system event that occurs at a given time point

   Name              | Value
 --------------------|-------------------------------------------------------
  `time`             | A floating point value for time event occurs
  `type`             | Event type (one of { "fault_on", "fault_off" })

   Name              | Value
 --------------------|-------------------------------------------------------
  `element_id`       | Index of the bus where the fault is applied (optional)

`element_id` is a zero-based index into the case file's bus list, resolved to
that bus's id. A root-level `fault_bus`, which names the bus by id, takes
precedence when both are given.

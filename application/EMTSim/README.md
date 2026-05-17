# EMTSim (Electromagnetic Transients Simulation)

EMTSim runs an electromagnetic-transients simulation defined by a *solver file*
(`*.solver.json`) and a *case file* (`*.case.json`). The case file specifies
the EMT system model; the solver file specifies the simulation environment —
time and numerical controls, generated outputs, scheduled runtime events, and
an optional reference comparison.

The solver-file and generated-output formats are documented below. The EMT
case-file format is documented in
[`GridKit/Model/EMT/README.md`](../../GridKit/Model/EMT/README.md).
The runtime event system (action vocabulary, dispatch model, schedule
semantics) is documented in [`EVENTS.md`](EVENTS.md).

## Usage

```sh
EMTSim <solver-file.solver.json>
```

Relative paths in `case_file`, `output.monitor`, `output.ida_stats`, and
`validation.reference_file` are resolved against the solver file's directory.
Absolute paths are used as given.

## Solver file format

Solver files are strict JSON objects: unknown keys are rejected. The root
object has this shape:

```text
{
  "format_version": 1,
  "case_file": "...",
  "solve": { ... },
  "output": { ... },
  "schedule": [ ... ],
  "validation": { ... }
}
```

### Top-level fields

Field            | Required | Description
-----------------|----------|------------------------------------------------------
`format_version` | yes      | Solver-file format version. Must be `1`.
`case_file`      | yes      | Path to the EMT case file.
`solve`          | yes      | Time and numerical solver controls.
`output`         | no       | Generated artifacts. Omitted outputs are disabled.
`schedule`       | no       | Time-ordered runtime events. May be empty or omitted.
`validation`     | no       | Reference comparison.

### Solve

Field          | Required | Default  | Description
---------------|----------|----------|------------------------------------------
`t0`           | no       | `0.0`    | Initial simulation time.
`tmax`         | yes      | —        | Final simulation time. Must exceed `t0`.
`dt`           | yes      | —        | Output step size per solve segment.
`rel_tol`      | no       | `1e-8`   | IDA relative tolerance.
`abs_tol`      | no       | `1e-8`   | IDA absolute tolerance.
`max_steps`    | no       | `200000` | IDA maximum internal steps.
`use_jacobian` | no       | `true`   | Enables the configured Jacobian path.

### Output

Field       | Required | Description
------------|----------|-------------------------------------------------
`monitor`   | no       | CSV monitor output path. EMTSim adds a monitor sink if the case has none, or retargets the sink if the case has exactly one. A case with multiple sinks is rejected as ambiguous.
`ida_stats` | no       | IDA statistics JSON output path.

### Schedule

The `schedule` field is an array of events. Each event has the shape:

Field    | Required           | Description
---------|--------------------|------------------------------------------------------
`time`   | yes                | Simulation time at which the event fires, in seconds. Must lie in `[t0, tmax]`.
`target` | yes                | Case `name` of the bus or component receiving the event.
`action` | yes                | Action name: `open`, `close`, `fault`, or `clear`.
`params` | if action needs it | Action payload. `fault` requires `params.r`; `params` is optional otherwise.

The action vocabulary, dispatch model, and schedule semantics (stable sort by
time, file order on ties, same-time batching) are documented in
[`EVENTS.md`](EVENTS.md).

### Validation

Field             | Required | Default | Description
------------------|----------|---------|------------------------------------------
`reference_file`  | yes      | —       | Reference CSV path.
`error_tolerance` | no       | `1e-4`  | Maximum accepted CSV difference.

Validation compares the generated monitor CSV against the reference and
requires a monitor output file. EMTSim exits `0` when the maximum error is
within `error_tolerance` and the solve succeeded, `1` otherwise.

## Generated output formats

### Monitor CSV

When `output.monitor` is set, EMTSim writes a comma-delimited CSV monitor
file.

Row            | Format
---------------|------------------------------------------------------
Header         | `t,<label>_<variable>,...`
Data row       | One numeric value per header column.
Time column    | `t`, in seconds.
Value columns  | Monitored bus/component variables selected by the case file.

EMTSim writes the initial state, each requested output time, and a post-event
state after each event batch. Event boundaries may therefore appear twice:
once for the pre-event integrated state, and once after the scheduled mutation
has been applied and IDA has been re-initialized.

### IDA JSON

When `output.ida_stats` is set, EMTSim writes IDA solver statistics as JSON.
The top-level statistics summarize the whole run. The `segments` array records
the same statistics for each solve segment between event batches.

Top-level field       | Type   | Description
----------------------|--------|-------------------------------------------------
`sundials`            | object | SUNDIALS metadata.
`integrator`          | object | Accumulated IDA integrator counters.
`nonlinear_solver`    | object | Accumulated nonlinear solver counters.
`linear_solver`       | object | Accumulated linear solver counters and final flag.
`final_state`         | object | IDA state at the end of the final segment.
`segment_count`       | int    | Number of entries in `segments`.
`segments`            | array  | Per-segment statistics.

`sundials` fields:

Field           | Type   | Description
----------------|--------|----------------------------
`version`       | string | SUNDIALS version used.
`logging_level` | int    | Compile-time SUNDIALS logging level.

Statistics object fields:

Section              | Fields
---------------------|-----------------------------------------------------
`integrator`         | `steps`, `residual_evals`, `linear_solver_setups`, `error_test_failures`, `backtrack_operations`
`nonlinear_solver`   | `iterations`, `convergence_failures`, `step_solve_failures`
`linear_solver`      | `jacobian_evals`, `last_jacobian_eval_step`, `jacobian_time`, `jacobian_cj`, `iterations`, `convergence_failures`, `residual_evals`, `preconditioner_evals`, `preconditioner_solves`, `jtimes_setup_evals`, `jtimes_evals`, `last_flag`, `last_flag_name`
`final_state`        | `last_order`, `current_order`, `actual_initial_step`, `last_step`, `current_step`, `current_time`, `current_cj`

Each `segments[]` entry has `start_time`, `end_time`, `output_steps`, and the
same `integrator`, `nonlinear_solver`, `linear_solver`, and `final_state`
objects. `start_time`/`end_time` are EMTSim segment boundaries; `final_state`
reports IDA's actual final internal state for that segment.

Compact example:

```json
{
  "sundials": { "version": "7.4.0", "logging_level": 2 },
  "segment_count": 2,
  "integrator": {
    "steps": 388,
    "residual_evals": 550,
    "linear_solver_setups": 43,
    "error_test_failures": 1,
    "backtrack_operations": 0
  },
  "nonlinear_solver": {
    "iterations": 550,
    "convergence_failures": 1,
    "step_solve_failures": 0
  },
  "linear_solver": {
    "jacobian_evals": 43,
    "last_jacobian_eval_step": 237,
    "jacobian_time": 0.01079,
    "jacobian_cj": 72364.58,
    "iterations": 0,
    "convergence_failures": 0,
    "residual_evals": 0,
    "preconditioner_evals": 0,
    "preconditioner_solves": 0,
    "jtimes_setup_evals": 0,
    "jtimes_evals": 0,
    "last_flag": 0,
    "last_flag_name": "IDALS_SUCCESS"
  },
  "final_state": {
    "last_order": 5,
    "current_order": 5,
    "actual_initial_step": 1.0e-11,
    "last_step": 2.3e-7,
    "current_step": 2.3e-7,
    "current_time": 0.0600002,
    "current_cj": 9751556.8
  },
  "segments": [
    {
      "start_time": 0.0,
      "end_time": 0.01,
      "output_steps": 100,
      "integrator": { "...": "same fields as top-level integrator" },
      "nonlinear_solver": { "...": "same fields as top-level nonlinear_solver" },
      "linear_solver": { "...": "same fields as top-level linear_solver" },
      "final_state": { "...": "same fields as top-level final_state" }
    }
  ]
}
```

## Example

```json
{
  "format_version": 1,
  "case_file": "TwoBus.case.json",

  "solve": {
    "tmax": 0.06,
    "dt": 1e-4,
    "rel_tol": 1e-8,
    "abs_tol": 1e-8,
    "max_steps": 200000,
    "use_jacobian": true
  },

  "output": {
    "monitor": "TwoBus.csv",
    "ida_stats": "TwoBus.ida.json"
  },

  "schedule": [
    { "time": 0.010, "target": "receiving_bus", "action": "fault", "params": { "r": 15.0, "phases": "abc" } },
    { "time": 0.011, "target": "receiving_bus", "action": "clear" },
    { "time": 0.011, "target": "load_breaker", "action": "open" }
  ],

  "validation": {
    "reference_file": "TwoBus.ref.csv",
    "error_tolerance": 1e-4
  }
}
```

This run faults `receiving_bus` through a 15 Ω resistance at `t = 0.010`, then
at `t = 0.011` clears the fault and trips `load_breaker` simultaneously, and
integrates to `t = 0.06`. The two events at `t = 0.011` batch into one IDA
re-initialization. The monitor CSV is written to `TwoBus.csv` and compared
against `TwoBus.ref.csv` with tolerance `1e-4`.

# EMTSim

EMTSim solves an EMT case from a `.solver.json` file. The solver file is the
small orchestration layer around an EMT case: it selects the case file, sets
time integration controls, chooses generated outputs, installs scheduled
events, and optionally validates the result against a reference CSV.

```text
EMTSim <file.solver.json>
        |
        v
+------------------+
| readSolverFile   |  .solver.json -> SolverFile
+--------+---------+
         |
         v
+------------------+       +------------------+
| SolverFile       | ----> | loadCase         |
| solve/output/... |       | EMT Case         |
+--------+---------+       +--------+---------+
         |                          |
         v                          v
+------------------+       +------------------+
| installSchedule  | ----> | SystemModel + IDA|
| names -> refs    |       | segmented solve  |
+------------------+       +--------+---------+
                                    |
                                    v
                        monitor output, IDA stats,
                        optional validation
```

## Usage

```sh
EMTSim TwoBus.solver.json
```

Relative paths inside the solver file are resolved relative to the directory
containing that solver file. This keeps examples and regression fixtures
self-contained.

## Solver File

The top-level sections are intentionally regular:

```text
format_version  file format version
case_file       EMT case file to solve
solve           time and numerical solver controls
output          generated artifacts
schedule        time-ordered runtime events
validation      optional reference comparison
```

Canonical ordering:

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
    "monitor": { "file": "TwoBusFault.csv" },
    "ida_stats": { "file": "TwoBusFault.ida-stats.json", "format": "json" }
  },

  "schedule": [
    { "time": 0.010, "target": "load_bus", "action": "fault", "params": { "r": 15.0, "phases": "abc" } },
    { "time": 0.011, "target": "load_bus", "action": "clear" },
    { "time": 0.011, "target": "load_breaker", "action": "open" }
  ],

  "validation": {
    "reference_file": "TwoBusFault.ref.csv",
    "error_tolerance": 1e-4
  }
}
```

## Sections

`format_version` is required and must be `1`.

`case_file` is required. It points to an EMT case JSON file.

`solve` is required. It contains numerical controls for one solve:

| Field | Required | Default | Meaning |
| --- | --- | --- | --- |
| `t0` | no | `0.0` | Initial simulation time. |
| `tmax` | yes | none | Final simulation time. |
| `dt` | yes | none | Output step size for each solve segment. |
| `rel_tol` | no | `1e-8` | IDA relative tolerance. |
| `abs_tol` | no | `1e-8` | IDA absolute tolerance. |
| `max_steps` | no | `200000` | IDA maximum internal steps. |
| `use_jacobian` | no | `true` | Enables the configured Jacobian path. |

`output` is optional. Omitted outputs are disabled.

| Field | Meaning |
| --- | --- |
| `monitor.file` | CSV monitor output path. If the case has no monitor sink, EMTSim adds one. If the case has one sink, EMTSim overrides it. Multiple existing sinks are rejected because the target is ambiguous. |
| `ida_stats.file` | IDA statistics output path. |
| `ida_stats.format` | `json` or `text`; defaults to `json`. |

`schedule` is optional. Events are applied in file order for events with the
same time. V1 targets EMT case names: bus names and component names.

| Action | Parameters | Intended target |
| --- | --- | --- |
| `fault` | `r` required; `x`, `percent`, and `phases` optional | Bus |
| `clear` | `phases` optional | Bus or supported components |
| `open` | `phases` optional | Switching components |
| `close` | `phases` optional | Switching components |

`phases` may be `a`, `b`, `c`, any combination such as `ab`, or `abc`. It
defaults to `abc`.

`validation` is optional. It compares the generated monitor CSV against a
reference CSV:

| Field | Required | Default | Meaning |
| --- | --- | --- | --- |
| `reference_file` | yes | none | Reference CSV path. |
| `error_tolerance` | no | `1e-4` | Maximum accepted CSV difference. |

## IDA Statistics

JSON statistics are written as:

```json
{
  "steps": 1200,
  "residual_evals": 1408,
  "linear_decompositions": 1200,
  "error_test_failures": 2,
  "nonlinear_iterations": 1401,
  "nonlinear_convergence_failures": 0
}
```

Text statistics use the existing `IdaStats::report()` output.

## Implementation Shape

`GridKit/Model/EMT/IO/SolverFile.hpp` owns the file contract:

```cpp
namespace GridKit::EMT::IO
{
  struct SolverFile
  {
    int format_version{1};
    std::filesystem::path path;
    std::filesystem::path case_file;
    Solve solve;
    Output output;
    std::vector<Event> schedule;
    std::optional<Validation> validation;
  };

  SolverFile readSolverFile(const std::filesystem::path& path);
}
```

The executable stays thin: load the solver file, load the EMT case, apply the
requested outputs, install the schedule, solve the segmented event timeline,
write optional IDA statistics, and run optional validation.

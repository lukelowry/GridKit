# Input file for GridKit phasor dynamics application

## Root elements

   Name               | Value
 ---------------------|-------------------------------------------------------
  `system_model_file` | Path to the system model file[^1]
  `dt`                | A nonnegative output sample interval; `0` outputs at solver-selected steps
  `tmax`              | A floating point value for max time
  `ida_max_order`     | Optional integer from 1 to 5 limiting IDA's maximum integrator order
  `ida_max_steps`     | Optional positive integer limiting IDA's maximum internal steps per solve call
  `ida_max_dt`        | Optional positive floating point value limiting IDA's internal max step
  `rel_tol`           | Optional positive scalar IDA relative tolerance; must be paired with `abs_tol`
  `abs_tol`           | Optional positive scalar IDA absolute tolerance; must be paired with `rel_tol`
  `mu`                | Optional positive CommonMath smoothing scale; defaults to `240`
  `events`            | An array of event groups (see [Events](#events) below)
  `output_file`       | Optional path to monitor output (CSV); omitted means monitors are disabled
  `ida_stats`         | Optional path to IDA aggregate statistics JSON
  `ida_steps`         | Optional path to IDA accepted-step JSON
  `reference_file`    | Optional path to a reference CSV used by validation tooling
  `error_tolerance`   | Optional floating point validation error tolerance

[^1]: See system model [case format](../../Model/PhasorDynamics/INPUT_FORMAT.md)

## Events

Each event group describes a system event that occurs at a given time point

   Name              | Value
 --------------------|-------------------------------------------------------
  `time`             | A floating point value for time event occurs
  `type`             | Event type (one of { "fault_on", "fault_off" })
  `element_id`       | An integer value referencing the element associated with the event (e.g., bus fault id)

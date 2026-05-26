# Input file for GridKit phasor dynamics application

## Root elements

   Name               | Value
 ---------------------|-------------------------------------------------------
  `system_model_file` | Path to the system model file[^1]
  `dt`                | A nonnegative output sample interval; `0` outputs at solver-selected steps
  `tmax`              | A floating point value for max time
  `ida_max_order`     | Optional integer from 1 to 5 limiting IDA's maximum integrator order
  `ida_max_dt`        | Optional positive floating point value limiting IDA's internal max step
  `events`            | An array of event groups (see [Events](#events) below)
  `output_file`       | Optional path to monitor output (CSV); omitted means monitors are disabled
  `ida_stats`         | Optional path to IDA aggregate statistics JSON
  `ida_steps`         | Optional path to IDA accepted-step JSON
  `reference_file`    | A string containing the name of the case
  `error_tolerance`   | A string containing the name of the case

[^1]: See system model [case format](../../Model/PhasorDynamics/INPUT_FORMAT.md)

## Events

Each event group describes a system event that occurs at a given time point

   Name              | Value
 --------------------|-------------------------------------------------------
  `time`             | A floating point value for time event occurs
  `type`             | Event type (one of { "fault_on", "fault_off" })
  `element_id`       | An integer value referencing the element associated with the event (e.g., bus fault id)

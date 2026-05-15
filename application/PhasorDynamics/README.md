# Input file for GridKit phasor dynamics application

## Root elements

   Name               | Value
 ---------------------|-------------------------------------------------------
  `system_model_file` | Path to the system model file[^1]
  `dt`                | A floating point value for time step size
  `tmax`              | A floating point value for max time
  `events`            | An array of event groups (see [Events](#events) below)
  `output_file`       | Path to output (CSV) file
  `ida_output_file`   | Optional path to structured IDA statistics JSON
  `ida_log_file`      | Optional path to SUNDIALS IDA warning/error log
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

## IDA diagnostics

When `ida_output_file` is present, PDSim writes the same structured IDA
statistics JSON used by EMTSim: a whole-run summary plus `segment_count` and
`segments[]` for each solve interval between event batches. When
`ida_output_file` is omitted, PDSim does not collect or write IDA statistics.

When `ida_log_file` is present, PDSim configures the SUNDIALS logger to write
IDA warnings and errors to that file. Relative `ida_output_file` and
`ida_log_file` paths are resolved against the solver file directory.

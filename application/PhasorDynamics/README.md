# Input file for GridKit phasor dynamics application

## Root elements

   Name               | Value
 ---------------------|-------------------------------------------------------
  `system_model_file` | Path to the system model file[^1]
  `dt`                | A floating point value for time step size
  `tmax`              | A floating point value for max time
  `events`            | An array of event groups (see [Events](#events) below)
  `output`            | Optional generated output paths (see [Output](#output) below)
  `reference_file`    | A string containing the name of the case
  `error_tolerance`   | A string containing the name of the case

[^1]: See system model [case format](../../Model/PhasorDynamics/INPUT_FORMAT.md)

## Output

The optional `output` object uses the same shape as EMTSim:

```json
{
  "output": {
    "monitor": "case.csv",
    "ida_stats": "case.ida.json"
  }
}
```

  Name        | Value
  ------------|-------------------------------------------------------
  `monitor`   | Optional path to monitored variable CSV output
  `ida_stats` | Optional path to structured IDA statistics JSON

Relative `output.monitor`, `output.ida_stats`, and `reference_file` paths are
resolved against the solver file directory.

## Events

Each event group describes a system event that occurs at a given time point

   Name              | Value
 --------------------|-------------------------------------------------------
  `time`             | A floating point value for time event occurs
  `type`             | Event type (one of { "fault_on", "fault_off" })
  `element_id`       | An integer value referencing the element associated with the event (e.g., bus fault id)

## IDA diagnostics

When `output.ida_stats` is present, PDSim writes the same structured IDA
statistics JSON used by EMTSim: a whole-run summary plus `segment_count` and
`segments[]` for each solve interval between event batches. When
`output.ida_stats` is omitted, PDSim does not collect or write IDA statistics.

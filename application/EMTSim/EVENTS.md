# Electromagnetic Transients Events

## Overview

This document describes the runtime event system for EMT component models. A
runtime event mutates the state of a single bus or component at a scheduled
simulation time. Events are listed in a solver file (see
[EMTSim](README.md)) and applied by the EMT `SystemModel` between integration
segments. Buses and components are addressed by their case-file `name`.

## Event

An event is four fields:

Field    | Description
---------|------------------------------------------------------
`time`   | Simulation time at which the event fires, in seconds
`target` | Case `name` of the bus or component receiving the event
`action` | The mutation to perform, drawn from the action vocabulary
`order`  | Listing index in the solver file; breaks ties between same-time events

```cpp
struct Event {
  double                         time;
  std::string                    target;
  GridKit::Model::Events::Action action;
  std::size_t                    order;
};
```

Buses and components share a single name namespace. `installSchedule` resolves
`target` against bus names first, then component names.

## Action vocabulary

`Action` is a `std::variant` of the action structs in
`GridKit::Model::Events` (`GridKit/Model/Events.hpp`). Each struct exposes a
canonical name as `static constexpr std::string_view name`.

C++ type        | JSON name | Params                        | Applies to
----------------|-----------|-------------------------------|------------
`Events::Open`  | `open`    | `phases`                      | `Breaker`
`Events::Close` | `close`   | `phases`                      | `Breaker`
`Events::Fault` | `fault`   | `r`, `x`, `percent`, `phases` | `Bus`
`Events::Clear` | `clear`   | `phases`                      | `Bus`

Every action carries a `PhaseMask phases` selecting which of phases `a`, `b`,
`c` it affects; it defaults to `abc`. `open`/`close` add or remove phases from
a breaker's closed set; `fault`/`clear` add or remove phases from a bus's
active-fault set.

`fault` additionally carries fault resistance `r`, reactance `x`, and a
position `percent`. `Bus`, the only fault target today, uses `r` only and
rejects a nonzero `x` or `percent`. Action semantics — how a target responds —
are owned by the receiving model; see the
[Bus](../../GridKit/Model/EMT/Bus/README.md) and Breaker model READMEs.

## Dispatch

EMT dispatch is resolved at compile time through `EventTraits<EntityT>`
(`GridKit/Model/EMT/System/Events.hpp`). A model type that handles events
specializes `EventTraits` with three members:

- `supports<ActionT>()` — whether the model accepts an action type
- `prepare(entity, action)` — one-time structural setup run before the solve
- `apply(entity, action)` — the state mutation run when the event fires

The unspecialized `EventTraits` reports `supports() == false` and throws from
`apply`. `Bus` specializes it for `Fault` and `Clear`; `Breaker` for `Open`
and `Close`.

`SystemModelData::schedule(time, target, action)` records an event as a
`ScheduledEvent` holding `validate`, `prepare`, and `apply` thunks bound to the
target. `installSchedule` (`GridKit/Model/EMT/IO/SolverFile.hpp`) walks the
solver-file schedule, looks each `target` up by name, and calls `schedule`
once per event.

## Schedule

`SystemModel::initializeEvents` copies the recorded events into the run
schedule, then:

1. validates every event time (finite, nonnegative),
2. `stable_sort`s the schedule by `time`, breaking ties by `order` so
   same-time events keep solver-file listing order,
3. runs each event's `validate` and `prepare` thunk once, up front.

During the solve, EMTSim integrates to `nextEventTime()`, calls
`applyNextEventBatch()`, and re-initializes IDA. A batch is every event whose
`time` is exactly equal (`double` equality) to the next event time; the batch
is applied in `order`, and the integrator is re-initialized once per batch.
When two same-time events target the same entity, the last-listed event wins.

## Example schedule

A schedule that faults `receiving_bus` at `t = 0.010`, clears it at
`t = 0.011`, and simultaneously opens `load_breaker`:

```json
"schedule": [
  { "time": 0.010, "target": "receiving_bus", "action": "fault", "params": { "r": 15.0, "phases": "abc" } },
  { "time": 0.011, "target": "receiving_bus", "action": "clear" },
  { "time": 0.011, "target": "load_breaker",  "action": "open"  }
]
```

The two events at `t = 0.011` apply in listing order in a single IDA
re-initialization.

## Errors

Failure                         | Source             | Message
--------------------------------|--------------------|--------
Unknown action string           | parser             | `unknown action '<str>'; valid actions are open, close, fault, clear`
`fault` missing `r`             | parser             | `params.r: required field missing`
Unknown key in `params`         | parser             | `params: unknown key '<key>'`
Event time past `tmax`          | parser             | `schedule[<i>].time: exceeds solve.tmax`
Event time before `t0`          | parser             | `schedule[<i>].time: precedes solve.t0`
Unknown target name             | `installSchedule`  | `unknown event target '<name>'`
Action not supported by target  | `initializeEvents` | `EMT bus does not support scheduled event action` / `EMT component does not support scheduled event action`

## Adding a new action

1. Add a struct to `GridKit::Model::Events` with a
   `static constexpr std::string_view name` and any payload fields, and add it
   to the `Action` variant alias.
2. Add a case to `readAction` (`GridKit/Model/EMT/IO/SolverFile.hpp`) mapping
   the JSON name to the struct, with key and value validation.
3. Specialize `EventTraits` for each receiving model so `supports`, `prepare`,
   and `apply` handle the new action.
4. Add a row to the action vocabulary table above, and document the action's
   effect in each receiving model's README.

# Adaptive Texas 2k and WECC profile — 2026-07-27

## Scope

This run profiles the restored Texas and WECC cases on branch
`lukel/merged-dev` at `e9b59511`, with the local instrumentation captured in
[`instrumentation.patch`](instrumentation.patch).

Both studies use adaptive IDA stepping, default tolerances (`1e-7` relative,
`1e-9` absolute), `dt_monitor = 1/240 s`, and a 20-second horizon. The process
was pinned to CPU 0. Texas variants use five trials because the host showed
substantial frequency variance; WECC variants use three.

The external wall time was recorded with:

```bash
taskset -c 0 /usr/bin/time \
  -f 'wall_seconds=%e\nuser_seconds=%U\nsys_seconds=%S\nmaxrss_kb=%M' \
  build/application/PhasorDynamics/DynamicSimulation <solver.json>
```

`raw/` contains every application profile and timing record. `variants/`
contains the exact modified case and solver inputs used for controlled runs.
The baseline inputs are the repository examples at `e9b59511`.

## Baseline results

| Case | States | Wall median | Wall range | Steps | Residuals | Jacobians | CSV bytes |
|---|---:|---:|---:|---:|---:|---:|---:|
| Texas | 24,352 | 7.55 s | 7.09–11.28 s | 2,156 | 2,938 | 162 | 549,045,895 |
| WECC | 4,426 | 0.72 s | 0.70–0.73 s | 1,668 | 2,067 | 86 | 86,041,983 |

Exclusive timers account for 99.3% of Texas wall time and 97.5% of WECC wall
time.

| Exclusive contributor | Texas | WECC |
|---|---:|---:|
| Residual callback | 2.321 s (30.7%) | 0.124 s (17.2%) |
| CSV formatting/write | 1.717 s (22.7%) | 0.337 s (46.8%) |
| IDA internal remainder | 1.126 s (14.9%) | 0.107 s (14.8%) |
| KLU solve | 1.094 s (14.5%) | 0.054 s (7.4%) |
| Jacobian callback | 0.530 s (7.0%) | 0.028 s (3.9%) |
| Monitor state update | 0.381 s (5.0%) | 0.039 s (5.4%) |
| KLU setup/refactor | 0.327 s (4.3%) | 0.014 s (1.9%) |

Texas residual-model leaders were Branch (0.675 s), BusFault (0.381 s), and
LoadZ (0.368 s). The single largest Texas runtime bucket is residual evaluation;
WECC is dominated by monitor formatting/output.

## Controlled variants

- `devnull`: preserves formatting and state updates but writes to `/dev/null`.
- `sinkless`: preserves selected monitor variables and state updates but has no
  output sink.
- `no-monitors`: removes selected monitor variables while retaining all 4,800
  `IDASolve` targets.
- `no-monitor-targets`: removes selected monitors and sets `dt_monitor = 0`.
  Texas failed this diagnostic at `t = 10.0848813902546` because one long
  `IDASolve` exceeded IDA's default 500-step limit. It is not included in the
  performance comparison.
- `one-fault-no-monitors`: Texas sensitivity case retaining only the BusFault
  used by the study schedule.

| Variant | Texas wall median | WECC wall median |
|---|---:|---:|
| Baseline CSV | 7.55 s | 0.72 s |
| `/dev/null` | 7.37 s | 0.60 s |
| Sinkless | 5.83 s | 0.34 s |
| No selected monitors | 5.28 s | 0.33 s |

All valid output controls retained identical SUNDIALS counts within each case.
Removing monitoring reduced wall time by 30.1% for Texas and 54.2% for WECC.

The Texas one-fault sensitivity reduced the state dimension from 24,352 to
20,354 and reduced the monitor-free application time from 5.236 s to 4.227 s
(19.3%). Residual time fell 22.4%, KLU solve time fell 15.1%, and peak RSS fell
12.4%. This is a performance sensitivity result, not a production case change.

Machine-readable medians are in [`summary.csv`](summary.csv).

## Measurement caveats

- `Complete in` is process CPU time; profile buckets and external elapsed time
  use wall clocks. Percentages therefore use external wall time.
- Residual, Jacobian, and KLU timers include work from both `IDACalcIC` and
  adaptive `IDASolve` calls.
- System residual-family timers include one allocation-time evaluation; the
  reported family times were scaled by `residual_calls/system_residual_calls`.
- `monitor_print_seconds` excludes the header and final stream close/flush.
- Texas and WECC differ in topology and model mix, so their ratio is supporting
  scaling evidence rather than a controlled complexity exponent.
- Generated `mon.csv` files are intentionally not retained.

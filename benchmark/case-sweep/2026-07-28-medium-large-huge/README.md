# Medium, Large and Huge case sweep, 2026-07-28

## Scope

Runtime of `DynamicSimulation` across every Medium, Large and Huge example
case, under two studies that differ only in horizon and fault timing. Branch
`lukel/merged-dev` at `0a68b99f`, plus the uncommitted GENROU algebraic
reduction described in [Notes](#notes).

Monitor output is disabled in both studies, so no file I/O sits inside the
measurement.

## Solver parameters

Identical for all six cases in a given study. Only the fault bus differs. No
study file sets an `ida` block, so every case takes the application defaults.

| Parameter | 20 s study | 10 s study |
|---|---|---|
| Horizon `tmax` | 20.0 s | 10.0 s |
| Fault applied | t = 10.0 s | t = 1.0 s |
| Fault cleared | t = 10.1 s | t = 1.1 s |
| Fault duration | 100 ms | 100 ms |
| `dt_monitor` | 1/240 s | 1/240 s |
| `IDASolve` targets | 4,800 | 2,400 |
| Monitor sinks | none | none |

| Parameter | Value | Source |
|---|---|---|
| Fault impedance | R = 0.0, X = 0.001 pu | `BusFault` params, identical in all six cases |
| System frequency base | 60.0 Hz | case header |
| System VA base | 100 MVA | case header |
| Integrator | SUNDIALS IDA, variable-order BDF, max order 5 | |
| Relative tolerance | 1e-7 | `DEFAULT_SOLVER_REL_TOL` |
| Absolute tolerance | 1e-9, scalar via `IDASStolerances` | `DEFAULT_SOLVER_ABS_TOL` |
| Linear solver | KLU sparse, analytic Jacobian via `IDASetJacFn` | |
| KLU ordering | AMD | GridKit default, overrides SUNDIALS COLAMD |
| `suppress_alg` | false, algebraic variables included in the error test | |
| Minimum and maximum step | unbounded | |
| Max internal steps per output | 500 | SUNDIALS default |

| Case | Tier | Fault bus |
|---|---|---:|
| hawaii | Medium | 1 |
| newengland | Medium | 2 |
| illinois | Large | 2 |
| wecc | Large | 3903 |
| texas | Large | 1027 |
| activsg10k | Huge | 10684 |

## Method

- Single CPU, pinned with `taskset -c 4`.
- Minimum over 5 interleaved trials. Cases rotate inside each round so host
  drift lands on all of them equally.
- Bucket timings come from the in-process `GRIDKIT_PROFILE` and
  `SYSTEM_RESIDUAL_PROFILE` blocks.
- `SUNDIALS internal` is `IDASolve` minus the four instrumented buckets. It
  covers the integrator's own vector algebra and output interpolation.
- Integration counters were identical across every trial in both studies, so
  the work is deterministic.

```bash
taskset -c 4 /usr/bin/time -f 'wall_seconds=%e\nmaxrss_kb=%M' \
  build/application/PhasorDynamics/DynamicSimulation <solver.json>
```

`raw/` holds every application profile and timing record. `variants/` holds the
solver inputs and the run scripts. Case files are regenerated from the
repository examples with `variants/strip_monitors.py` rather than stored here,
because they are large.

---

# Study A, 20 s horizon, fault at 10.0 s to 10.1 s

## System size and integration work

| Case | Tier | Buses | Devices | Signals | States | Jac nnz | nnz/N | Steps | Residuals | Jacobians | Linear setups | Nonlin iters | Err-test fails | Nonlin fails | Resid/step | Resid/Jac |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| hawaii | Medium | 37 | 235 | 117 | 778 | 4,052 | 5.2 | 2,757 | 3,803 | 249 | 249 | 3,797 | 103 | 2 | 1.38 | 15.3 |
| newengland | Medium | 39 | 108 | 40 | 340 | 1,612 | 4.7 | 6,464 | 9,811 | 972 | 972 | 9,805 | 546 | 20 | 1.52 | 10.1 |
| illinois | Large | 200 | 531 | 120 | 882 | 5,656 | 6.4 | 1,208 | 1,557 | 129 | 129 | 1,551 | 31 | 9 | 1.29 | 12.1 |
| wecc | Large | 243 | 1,661 | 670 | 4,420 | 18,752 | 4.2 | 2,058 | 2,898 | 197 | 197 | 2,890 | 99 | 1 | 1.41 | 14.7 |
| texas | Large | 2,000 | 5,992 | 1,278 | 11,724 | 67,198 | 5.7 | 1,864 | 2,528 | 148 | 148 | 2,522 | 61 | 0 | 1.36 | 17.1 |
| activsg10k | Huge | 10,000 | 37,997 | 7,404 | 71,086 | 318,519 | 4.5 | 2,466 | 3,488 | 210 | 210 | 3,482 | 104 | 15 | 1.41 | 16.6 |

## Runtime breakdown, seconds

| Case | Wall | IDASolve | Consistent IC | Residual | Jacobian | KLU solve | KLU refactor | SUNDIALS internal | Peak RSS MB |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| hawaii | 0.070 | 0.063 | 0.001 | 0.019 | 0.013 | 0.013 | 0.004 | 0.015 | 10.9 |
| newengland | 0.080 | 0.074 | 0.001 | 0.020 | 0.015 | 0.015 | 0.006 | 0.018 | 10.1 |
| illinois | 0.040 | 0.036 | 0.002 | 0.009 | 0.007 | 0.008 | 0.004 | 0.009 | 11.8 |
| wecc | 0.340 | 0.323 | 0.008 | 0.103 | 0.044 | 0.062 | 0.019 | 0.096 | 18.9 |
| texas | 0.950 | 0.878 | 0.022 | 0.176 | 0.089 | 0.246 | 0.120 | 0.248 | 40.4 |
| activsg10k | 12.060 | 11.561 | 0.145 | 3.391 | 0.935 | 3.001 | 0.698 | 3.536 | 195.5 |

## Share of IDASolve, percent

| Case | Residual | Jacobian | KLU solve | KLU refactor | SUNDIALS internal |
|---|---:|---:|---:|---:|---:|
| hawaii | 29.4 | 19.9 | 20.9 | 6.6 | 23.2 |
| newengland | 27.0 | 20.6 | 20.2 | 8.1 | 24.1 |
| illinois | 24.3 | 19.8 | 21.5 | 10.3 | 24.1 |
| wecc | 31.7 | 13.7 | 19.2 | 5.7 | 29.7 |
| texas | 20.0 | 10.2 | 28.0 | 13.6 | 28.2 |
| activsg10k | 29.3 | 8.1 | 26.0 | 6.0 | 30.6 |

## Work-normalized cost

| Case | States | us/residual | us/Jacobian | us/KLU solve | us/KLU refactor | ns per residual per state | us/step |
|---|---:|---:|---:|---:|---:|---:|---:|
| hawaii | 778 | 4.9 | 50.3 | 3.4 | 16.6 | 6.3 | 22.8 |
| newengland | 340 | 2.0 | 15.6 | 1.5 | 6.1 | 6.0 | 11.4 |
| illinois | 882 | 5.7 | 55.6 | 5.0 | 29.0 | 6.4 | 30.0 |
| wecc | 4,420 | 35.4 | 224.3 | 21.4 | 94.1 | 8.0 | 157.1 |
| texas | 11,724 | 69.6 | 602.3 | 97.2 | 808.4 | 5.9 | 471.0 |
| activsg10k | 71,086 | 972.2 | 4,450.4 | 860.4 | 3,325.3 | 13.7 | 4,688.1 |

## Residual sweep by device family, seconds

| Case | Network SpMV | Generator | Exciter | Governor | Stabilizer | Converter | Fault |
|---|---:|---:|---:|---:|---:|---:|---:|
| hawaii | 0.000 | 0.006 | 0.005 | 0.005 | 0.000 | 0.000 | 0.000 |
| newengland | 0.001 | 0.004 | 0.003 | 0.003 | 0.003 | 0.000 | 0.000 |
| illinois | 0.001 | 0.002 | 0.002 | 0.002 | 0.000 | 0.000 | 0.000 |
| wecc | 0.002 | 0.015 | 0.009 | 0.017 | 0.001 | 0.051 | 0.000 |
| texas | 0.015 | 0.062 | 0.046 | 0.033 | 0.000 | 0.000 | 0.000 |
| activsg10k | 0.289 | 0.970 | 0.822 | 0.570 | 0.433 | 0.000 | 0.005 |

## Trial spread

| Case | Wall min | Wall max | Ratio |
|---|---:|---:|---:|
| hawaii | 0.070 | 0.070 | 1.00 |
| newengland | 0.080 | 0.080 | 1.00 |
| illinois | 0.040 | 0.040 | 1.00 |
| wecc | 0.340 | 0.370 | 1.09 |
| texas | 0.950 | 1.070 | 1.13 |
| activsg10k | 12.060 | 12.350 | 1.02 |

---

# Study B, 10 s horizon, fault at 1.0 s to 1.1 s

## System size and integration work

| Case | Tier | Buses | Devices | Signals | States | Jac nnz | Steps | Residuals | Jacobians | Linear setups | Nonlin iters | Err-test fails | Nonlin fails | Resid/step | Resid/Jac |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| hawaii | Medium | 37 | 235 | 117 | 778 | 4,052 | 2,599 | 3,591 | 261 | 261 | 3,585 | 107 | 3 | 1.38 | 13.8 |
| newengland | Medium | 39 | 108 | 40 | 340 | 1,612 | 6,239 | 9,590 | 1,009 | 1,009 | 9,584 | 561 | 20 | 1.54 | 9.5 |
| illinois | Large | 200 | 531 | 120 | 882 | 5,656 | 1,136 | 1,472 | 126 | 126 | 1,466 | 34 | 6 | 1.30 | 11.7 |
| wecc | Large | 243 | 1,661 | 670 | 4,420 | 18,752 | 2,339 | 3,260 | 175 | 175 | 3,252 | 88 | 2 | 1.39 | 18.6 |
| texas | Large | 2,000 | 5,992 | 1,278 | 11,724 | 67,198 | 1,754 | 2,433 | 151 | 151 | 2,427 | 64 | 0 | 1.39 | 16.1 |
| activsg10k | Huge | 10,000 | 37,997 | 7,404 | 71,086 | 318,519 | 2,208 | 3,210 | 218 | 218 | 3,204 | 105 | 17 | 1.45 | 14.7 |

## Runtime breakdown, seconds

| Case | Wall (s) | IDASolve | Consistent IC | Residual | Jacobian | KLU solve | KLU refactor | SUNDIALS internal | Peak RSS MB |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| hawaii | 0.060 | 0.060 | 0.001 | 0.018 | 0.013 | 0.012 | 0.004 | 0.013 | 10.9 |
| newengland | 0.070 | 0.073 | 0.001 | 0.020 | 0.016 | 0.015 | 0.006 | 0.017 | 10.0 |
| illinois | 0.040 | 0.034 | 0.002 | 0.009 | 0.007 | 0.007 | 0.004 | 0.007 | 11.8 |
| wecc | 0.370 | 0.343 | 0.008 | 0.118 | 0.041 | 0.069 | 0.017 | 0.097 | 18.9 |
| texas | 0.910 | 0.834 | 0.022 | 0.172 | 0.093 | 0.238 | 0.121 | 0.210 | 40.4 |
| activsg10k | 11.490 | 10.974 | 0.149 | 3.151 | 0.995 | 3.018 | 0.758 | 3.051 | 195.5 |

## Share of IDASolve, percent

| Case | Residual | Jacobian | KLU solve | KLU refactor | SUNDIALS internal |
|---|---:|---:|---:|---:|---:|
| hawaii | 29.5 | 21.3 | 20.6 | 6.9 | 21.8 |
| newengland | 26.9 | 21.8 | 19.9 | 8.7 | 22.6 |
| illinois | 25.2 | 20.6 | 22.0 | 11.1 | 21.0 |
| wecc | 34.6 | 11.8 | 20.2 | 5.1 | 28.3 |
| texas | 20.7 | 11.2 | 28.5 | 14.5 | 25.2 |
| activsg10k | 28.7 | 9.1 | 27.5 | 6.9 | 27.8 |

## Work-normalized cost

| Case | States | us/residual | us/Jacobian | us/KLU solve | us/KLU refactor | ns per residual per state | us/step |
|---|---:|---:|---:|---:|---:|---:|---:|
| hawaii | 778 | 4.9 | 48.8 | 3.4 | 15.9 | 6.3 | 23.0 |
| newengland | 340 | 2.0 | 15.8 | 1.5 | 6.3 | 6.0 | 11.7 |
| illinois | 882 | 5.8 | 55.8 | 5.1 | 30.0 | 6.6 | 30.0 |
| wecc | 4,420 | 36.3 | 231.8 | 21.2 | 99.0 | 8.2 | 146.5 |
| texas | 11,724 | 70.8 | 616.8 | 97.7 | 798.4 | 6.0 | 475.5 |
| activsg10k | 71,086 | 981.7 | 4,562.6 | 940.3 | 3,478.8 | 13.8 | 4,969.9 |

## Residual sweep by device family, seconds

| Case | Network SpMV | Generator | Exciter | Governor | Stabilizer | Converter | Fault |
|---|---:|---:|---:|---:|---:|---:|---:|
| hawaii | 0.000 | 0.005 | 0.005 | 0.005 | 0.000 | 0.000 | 0.000 |
| newengland | 0.001 | 0.003 | 0.003 | 0.003 | 0.003 | 0.000 | 0.000 |
| illinois | 0.001 | 0.002 | 0.002 | 0.002 | 0.000 | 0.000 | 0.000 |
| wecc | 0.002 | 0.018 | 0.010 | 0.020 | 0.001 | 0.059 | 0.000 |
| texas | 0.014 | 0.062 | 0.044 | 0.031 | 0.000 | 0.000 | 0.000 |
| activsg10k | 0.283 | 0.904 | 0.752 | 0.539 | 0.401 | 0.000 | 0.004 |

## Trial spread

| Case | Wall min | Wall max | Ratio |
|---|---:|---:|---:|
| hawaii | 0.060 | 0.070 | 1.17 |
| newengland | 0.070 | 0.080 | 1.14 |
| illinois | 0.040 | 0.040 | 1.00 |
| wecc | 0.370 | 0.480 | 1.30 |
| texas | 0.910 | 0.920 | 1.01 |
| activsg10k | 11.490 | 12.120 | 1.05 |

---

# Study A against Study B

| Case | Wall 20 s | Wall 10 s | Ratio | Steps 20 s | Steps 10 s | Ratio |
|---|---:|---:|---:|---:|---:|---:|
| hawaii | 0.070 | 0.060 | 0.86 | 2,757 | 2,599 | 0.94 |
| newengland | 0.080 | 0.070 | 0.88 | 6,464 | 6,239 | 0.97 |
| illinois | 0.040 | 0.040 | 1.00 | 1,208 | 1,136 | 0.94 |
| wecc | 0.340 | 0.370 | 1.09 | 2,058 | 2,339 | 1.14 |
| texas | 0.950 | 0.910 | 0.96 | 1,864 | 1,754 | 0.94 |
| activsg10k | 12.060 | 11.490 | 0.95 | 2,466 | 2,208 | 0.90 |

## Findings

- Halving the horizon removes only 3 to 10 percent of the steps in five of six
  cases. The post-fault transient dominates the step budget and the quiescent
  tail is close to free.
- wecc is the exception and takes 14 percent more steps on the shorter run.
  Faulting at t = 1.0 s catches it before it has settled from initialization.
- Per-call and per-state costs are unchanged between studies, so the profile is
  a property of the system rather than the horizon.
- Step count rather than size sets runtime below roughly 10,000 states.
  illinois is 2.6 times larger than hawaii and runs in half the time.
- Residual cost per state is flat near 6 ns from 340 to 11,724 states, then
  rises to 13.7 at 71,086. That is a cache effect rather than algorithmic
  scaling.
- newengland is the smallest system and the slowest of the three small cases.
  Its 546 error-test failures and resid/Jac ratio near 10 indicate the Newton
  solver refactors constantly. Its cost is integration difficulty, not size.
- texas is the only KLU-bound case at 41.6 percent combined solve and
  refactor, and it has the highest nnz per state at 5.7.
- wecc is converter-heavy. Its converter family is its largest residual
  contributor and exceeds its generators.
- activsg10k is the only case where SUNDIALS internal vector work is the
  largest single bucket.

## Notes

- Branch and load residual families read zero because their constant
  admittance is pre-assembled into the network Y-bus and evaluated as one
  sparse matrix-vector product rather than swept per device.
- The three small cases land near the 10 ms granularity of `/usr/bin/time`.
  Prefer their work-normalized columns over their wall times.
- The build carries an uncommitted GENROU change that evaluates the machine's
  algebraic quantities inline instead of carrying them as unknowns, taking
  GENROU from 15 states to 6. It reduces activsg10k from 79,420 to 71,086
  states. That formulation is build-sensitive and its activsg10k step count
  moves by a few percent on recompilation, so treat the activsg10k rows as
  accurate for this binary rather than reproducible across rebuilds. The other
  five cases are unaffected.

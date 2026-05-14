# **EMT Tiny TwoBus**

## Case Description

This case is a two-bus EMT example with a voltage source, a lumped-constant three-phase line, an RL load, CSV monitors, and a bus-resident three-phase fault scheduled from C++.

Model | Count
---|---
[Bus](../../../../GridKit/Model/EMT/Bus/README.md) | 2
[VoltageSource](../../../../GridKit/Model/EMT/Component/VoltageSource/README.md) | 1
[BranchLumpedConstant](../../../../GridKit/Model/EMT/Component/Branch/BranchLumpedConstant/README.md) | 1
[LoadRL](../../../../GridKit/Model/EMT/Component/LoadRL/README.md) | 1

## Data Notes

The line uses IEEE 13-node test feeder line code 601 data converted to SI per-meter `r`, `l`, and `c` matrices. The source data is the OpenDSS/EPRI IEEE test-case line-code file:
https://github.com/dss-extensions/electricdss-tst/blob/master/Version8/Distrib/IEEETestCases/IEEELineCodes.DSS

The nonzero `g` matrix is derived from the capacitance matrix with a small dielectric loss tangent of `1e-3`.

## Events

The following event type is provided for this case.

- Bus fault

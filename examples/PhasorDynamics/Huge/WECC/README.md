# Huge WECC Case

## Case Description

The case JSON header identifies this case as `ACTIVSg10k`. It contains 10,000
buses and a 5 s bus-fault simulation configured by `wecc.solver.json`.

Model | Count
---|---
[Bus](../../../../GridKit/Model/PhasorDynamics/Bus/README.md) | 10,000
[Branch](../../../../GridKit/Model/PhasorDynamics/Branch/README.md) | 12,706
[BusFault](../../../../GridKit/Model/PhasorDynamics/BusFault/README.md) | 10,000
[LoadZIP](../../../../GridKit/Model/PhasorDynamics/Load/LoadZIP/README.md) | 4,722
[GENROU](../../../../GridKit/Model/PhasorDynamics/SynchronousMachine/GENROUwS/README.md) | 926
[GENSAL](../../../../GridKit/Model/PhasorDynamics/SynchronousMachine/GENSALwS/README.md) | 530
[TGOV1](../../../../GridKit/Model/PhasorDynamics/Governor/Tgov1/README.md) | 926
HYGOV | 530
[IEEET1](../../../../GridKit/Model/PhasorDynamics/Exciter/IEEET1/README.md) | 1,333
ESDC1A | 123
[IEEEST](../../../../GridKit/Model/PhasorDynamics/Stabilizer/IEEEST/README.md) | 1,456
[SignalNode](../../../../GridKit/Model/PhasorDynamics/SignalNode/README.md) | 5,824

## Data Notes

This case requires HYGOV and ESDC1A model support.

## Events

The solver applies a bus fault at `t = 1.0 s` and clears it at `t = 1.1 s`.

## Outstanding

- Ensure HYGOV support is available in the target branch.
- Ensure ESDC1A support is available in the target branch.

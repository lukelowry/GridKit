# Huge WECC Case

## Case Description

The case JSON header identifies this case as `ACTIVSg10k`. It contains 10,000
buses and runs a 6 s bus-fault simulation configured by `wecc.solver.json`.
The model contains 23,253 non-bus devices and 5,824 signal nodes. Generator
speed is recorded every 0.02 s for all 1,456 synchronous machines.

Model | Count
---|---
[Bus](../../../../GridKit/Model/PhasorDynamics/Bus/README.md) | 10,000
[Branch](../../../../GridKit/Model/PhasorDynamics/Branch/README.md) | 12,706
[BusFault](../../../../GridKit/Model/PhasorDynamics/BusFault/README.md) | 1
[LoadZIP](../../../../GridKit/Model/PhasorDynamics/Load/LoadZIP/README.md) | 4,722
[GENROU](../../../../GridKit/Model/PhasorDynamics/SynchronousMachine/GENROU/README.md) | 926
[GENSAL](../../../../GridKit/Model/PhasorDynamics/SynchronousMachine/GENSAL/README.md) | 530
[TGOV1](../../../../GridKit/Model/PhasorDynamics/Governor/Tgov1/README.md) | 926
[HYGOV](../../../../GridKit/Model/PhasorDynamics/Governor/HYGOV/README.md) | 530
[IEEET1](../../../../GridKit/Model/PhasorDynamics/Exciter/IEEET1/README.md) | 1,333
[ESDC1A](../../../../GridKit/Model/PhasorDynamics/Exciter/ESDC1A/README.md) | 123
[IEEEST](../../../../GridKit/Model/PhasorDynamics/Stabilizer/IEEEST/README.md) | 1,456
[SignalNode](../../../../GridKit/Model/PhasorDynamics/SignalNode/README.md) | 5,824

## Data Notes

Bus nominal voltages use `params.kv` in kilovolts. HYGOV devices receive their
bus association through the connected synchronous machine and therefore do not
carry a `ports.bus` entry.

## Events

The solver applies a bus fault at `t = 0.1 s` and clears it at `t = 0.2 s`.

## Plotting

After running the case from this directory, generate a plot of all generator
speed traces with:

```bash
MPLBACKEND=Agg MPLCONFIGDIR=/tmp/gridkit-matplotlib \
python3 plot_wecc_monitors.py \
  --input mon.csv --solver wecc.solver.json \
  --output wecc.generator_omega.png
```

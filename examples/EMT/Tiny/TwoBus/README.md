# EMT Tiny TwoBus

This example runs a two-bus EMT system with a voltage source on `source_bus`,
an RL load on `receiving_bus`, and a lumped constant branch between them. A
three-phase shunt fault is applied at `receiving_bus` at `t = 1.0 s` and cleared
at `t = 1.1 s`.

```bash
cmake --build build --target EMTTinyTwoBus
python3 scripts/emtsim/twobus.py
```

The monitor CSV is written beside the built JSON files, and the plot is written
under `scripts/emtsim/output/`.

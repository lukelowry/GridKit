# PhasorDynamics Validation Tools

Lean generated-data tooling for validation studies. The scripts write temporary
solver JSON files and outputs under `build/validation/` by default, so tracked
examples stay untouched.

## Run Data

```bash
cmake --build build --target DynamicSimulation ContingencyAnalysis -j 10

python3 paper/cli.py run --cases newengland --steps --output-csv
python3 paper/cli.py run --cases newengland --contingency
python3 paper/cli.py tol-sweep --cases newengland --rtols 1e-3 1e-5 1e-7
python3 paper/cli.py mu-sweep --cases newengland --mu-values 60 120 240 480
```

Large contingency sweeps are opt-in when running all cases. Select a large case
explicitly or pass `--all-sweeps`.

## Plot Data

```bash
python3 paper/cli.py plot step-size --cases newengland
python3 paper/cli.py plot contingency --cases newengland --include-failed
python3 paper/cli.py plot ctg-frontier --cases newengland --include-failed
python3 paper/cli.py plot cumulative-work --cases newengland
python3 paper/cli.py plot work-per-step --cases newengland
python3 paper/cli.py plot tolerance --cases newengland
python3 paper/cli.py plot tol-step-size --cases newengland
python3 paper/cli.py plot mu --cases newengland
python3 paper/cli.py plot signals --cases newengland --quantity voltage
python3 paper/cli.py table errors
```

Figures and generated tables are written under `build/validation/figures/`.

## Fault Sweep Model Copies

For cases that ship only a placeholder `BusFault`, generate a copied study with
one fault per bus:

```bash
python3 paper/cli.py fault-sweep --cases wecc
python3 paper/cli.py fault-sweep --cases wecc --run
```

The generated case and solver JSON are under `build/validation/wecc/`.

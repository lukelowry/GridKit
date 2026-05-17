# `.fit.json` Sidecar Feature Notes

This folder exercises the GridKit EMT `.fit.json` sidecar feature with
OpenLine-generated characteristic-admittance fits.

## Implemented Shape

The EMT case remains readable and strict:

```json
"params": {
  "length": 150000.0,
  "phase_order": ["a", "b", "c"],
  "yc": {
    "fit_file": "fits/ol_345_horizontal_acsr_twin_transposed.yc.fit.json",
    "sha256": "b913334383f4cf4befeb0a060154813bc69ce75f8a997b2df366c0f8f161266c"
  }
}
```

The fit file owns the dense pole/residue arrays. The case file owns topology,
component names, port wiring, monitors, and human-readable operating intent.
There is no root-level `model_library` key.

## Runtime Consumer

`BranchFrequencyDependent` is the first consumer:

- two electrical ports, `from` and `to`
- one loaded phase-domain characteristic-admittance `Yc(s)` fit
- one rational realization per electrical port
- dynamic variable/equation counts from the rational state count
- port current monitors `ifa`, `ifb`, `ifc`, `ita`, `itb`, `itc`

Version 1 is intentionally `Yc`-only. It rejects a `propagation` parameter so a
case cannot imply distributed delay/history behavior that the runtime does not
yet implement.

## Files

```text
case_format_proposal.json
fits/
  ol_345_horizontal_acsr_twin_transposed.yc.fit.json
  ...
fit_manifest.json
validation_report.json
```

`fit_manifest.json` is optional for GridKit runtime. It is a human/catalog aid
that maps model IDs to sidecar files, SHA-256 hashes, source OpenLine metadata,
and validation summaries.

## Schema

The formal runtime schema is documented at:

```text
GridKit/Model/EMT/IO/FIT_JSON.md
```

Each `.fit.json` file stores one rational matrix transfer function:

```text
F(s) = D + sE + sum(R/(s-p))
```

For these examples:

- `quantity == "characteristic_admittance"`
- `domain == "phase"`
- `phase_order == ["a", "b", "c"]`
- `matrix_layout == "row_major"`
- `transfer_function.s_units == "rad/s"`
- `units.transfer == "S"`

## Generator Output

`fit_yc_rational.py` writes:

```text
fits/{model_id}.yc.fit.json
fit_manifest.json
validation_report.json
```

The generated sidecar bytes are hashed after writing, and the hash is recorded
in `fit_manifest.json`. The example case references the real sidecar and hash.

## Validation

The generated OpenLine set currently contains 10 passing models. The fitter
checks holdout and all-sample fit error, reciprocity, and positive-real sampled
band behavior. GridKit-side tests cover parser validation, hash mismatch,
relative path resolution, case construction, and `BranchFrequencyDependent`
current-injection signs for constant and zero fits.

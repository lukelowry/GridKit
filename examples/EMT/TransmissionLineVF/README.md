# EMT Transmission-Line Characteristic-Admittance Fits

This folder contains OpenLine-generated starter data for frequency-dependent
EMT transmission-line work. Each generated sidecar stores one phase-domain
characteristic admittance matrix:

```text
Yc(s) = Z(s)^-1 * sqrt(Z(s) * Y(s))
```

That convention matches `openline-compute` and is the side convention consumed
by GridKit EMT `BranchFrequencyDependent`.

## Files

| File | Purpose |
| --- | --- |
| `openline_yc_samples.csv` | Raw OpenLine `Yc(jw)` samples for 10 line variants, 1 Hz to 1 MHz. |
| `fits/*.yc.fit.json` | One GridKit EMT rational-fit sidecar per line. |
| `fit_manifest.json` | Catalog of generated sidecars with SHA-256 hashes and summary metrics. |
| `validation_report.json` | Holdout and all-sample fit quality metrics. |
| `fit_yc_rational.py` | Re-runnable fitter/validator for the sample CSV. |
| `case_format_proposal.json` | Example EMT case shape referencing one sidecar. |
| `FIT_JSON_FEATURE_PLAN.md` | Design notes for the `.fit.json` sidecar feature. |

The OpenLine-side sample generator is local and uncommitted at:

```text
C:\Users\wyattluke.lowery\Documents\GitHub\openline\crates\openline-compute\src\bin\gridkit-yc-samples.rs
```

## Generation

From the `openline` repo:

```powershell
cargo run -q -p openline-compute --bin gridkit-yc-samples -- `
  --f-min 1 --f-max 1000000 --points 801 `
  --output C:\Users\wyattluke.lowery\Documents\GitHub\GridKit\examples\EMT\TransmissionLineVF\openline_yc_samples.csv
```

From the `GridKit` repo:

```powershell
python examples\EMT\TransmissionLineVF\fit_yc_rational.py `
  --samples examples\EMT\TransmissionLineVF\openline_yc_samples.csv `
  --output examples\EMT\TransmissionLineVF\fit_manifest.json `
  --report examples\EMT\TransmissionLineVF\validation_report.json
```

The fitter writes `fits/{model_id}.yc.fit.json`, `fit_manifest.json`, and
`validation_report.json`.

## Fit Representation

Each `.fit.json` sidecar follows
`GridKit/Model/EMT/IO/FIT_JSON.md` and stores a
`GridKit::EMT::Math::RationalApproxData<RealT>` layout:

```text
F(s) ~= D + sum_k R_k / (s - p_k)
```

For this generated set all poles are stable real poles (`p_k < 0`, units
rad/s). `e`, `pair_real`, `pair_imag`, `pair_residue_real`, and
`pair_residue_imag` are present but empty/zero.

## Validation Gates

The fitter uses every other frequency point for fitting and the interleaved
points as holdout validation. For each line it reports:

- Relative Frobenius RMSE.
- Maximum and p95 relative Frobenius error by frequency.
- Maximum absolute entry error in siemens.
- Positive-real check: minimum eigenvalue of `(Yc + Yc^H) / 2`.
- Reciprocity check: `||Yc - Yc^T|| / ||Yc||`.

Current generated set:

```text
models: 10
pass:   10
holdout max relative error: 1.562e-2
holdout max relative RMSE:  1.145e-2
minimum fitted PR eig:      5.506e-5 S
```

These are useful starter models for GridKit EMT integration tests and API work.
They are not passivity-enforced vector-fitting outputs; they are passivity
checked on the sampled band. If future EMT line work needs stricter broadband
guarantees, the next step is OpenLine-side vector fitting with positive-real
enforcement for `Yc`.

## Case Usage

The EMT case references a sidecar from `params.yc`. Relative paths are resolved
against the case-file directory:

```json
{
  "name": "line",
  "class": "BranchFrequencyDependent",
  "params": {
    "length": 150000.0,
    "phase_order": ["a", "b", "c"],
    "yc": {
      "fit_file": "fits/ol_345_horizontal_acsr_twin_transposed.yc.fit.json",
      "sha256": "b913334383f4cf4befeb0a060154813bc69ce75f8a997b2df366c0f8f161266c"
    }
  },
  "ports": {
    "from": "source_bus",
    "to": "receiving_bus"
  },
  "mon": ["ifa", "ifb", "ifc", "ita", "itb", "itc"]
}
```

`BranchFrequencyDependent` v1 consumes characteristic admittance only. It rejects
a `propagation` parameter until propagation-function/delay history support is
implemented.

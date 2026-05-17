# EMT Rational-Fit Sidecar Format

`.fit.json` files store dense rational approximation data outside the readable
EMT case file. The EMT case owns topology and component parameters; each
sidecar owns one numeric transfer-function fit.

Format version 1 supports phase-domain transmission-line characteristic
admittance, `Yc(s)`, for `BranchFrequencyDependent`.

## File Shape

Each file is a JSON object with this structure:

```json
{
  "schema": "gridkit.emt.rational_fit",
  "format_version": 1,
  "fit_name": "ol_345_horizontal_acsr_twin_transposed_yc",
  "quantity": "characteristic_admittance",
  "domain": "phase",
  "realization": "pole_residue_real_sections",
  "matrix_layout": "row_major",
  "phase_order": ["a", "b", "c"],
  "transfer_function": {
    "variable": "s",
    "s_units": "rad/s",
    "form": "D + sE + sum(R/(s-p))"
  },
  "units": {
    "transfer": "S",
    "d": "S",
    "e": "S*s",
    "poles": "rad/s",
    "residues": "S/s"
  },
  "source": {},
  "rational_approx": {
    "dimension": 3,
    "d": [],
    "e": [],
    "real_poles": [],
    "real_residues": [],
    "pair_real": [],
    "pair_imag": [],
    "pair_residue_real": [],
    "pair_residue_imag": []
  },
  "validation": {},
  "metadata": {}
}
```

Unknown top-level fields are invalid, except arbitrary nested fields may appear
inside `metadata`. Unknown fields inside the required structural objects are
also invalid.

## Transfer Function

The numeric transfer function is

```text
F(s) = D + sE + sum_k R_k / (s - p_k)
```

For a complex conjugate pole pair `p = a +/- j*w`, the sidecar stores the real
section:

```text
pair_real[k] = a
pair_imag[k] = w
pair_residue_real[k] = real(R)
pair_residue_imag[k] = imag(R)
```

`pair_imag` stores only the positive imaginary frequency. The conjugate section
is implied by the real realization.

## Matrix Storage

All matrices are dense and row-major. For `dimension = n`, `d` and `e` each
contain `n*n` numbers:

```text
[m00, m01, ..., m0n, m10, m11, ...]
```

`real_residues` contains one row-major `n*n` matrix per real pole, in the same
order as `real_poles`. `pair_residue_real` and `pair_residue_imag` contain one
row-major matrix per conjugate-pair section.

For EMT v1 characteristic admittance files:

- `dimension` must be `3`.
- `phase_order` must be `["a", "b", "c"]`.
- `quantity` must be `"characteristic_admittance"`.
- `domain` must be `"phase"`.
- `realization` must be `"pole_residue_real_sections"`.
- `matrix_layout` must be `"row_major"`.
- `transfer_function.variable` must be `"s"`.
- `transfer_function.s_units` must be `"rad/s"`.
- `units.transfer` must be `"S"`.

## Stability And Numbers

Every numeric entry must be finite. Real poles and the real part of every
complex-pair section must be strictly negative. `pair_imag` entries must be
strictly positive.

The parser also checks array lengths against `dimension` and constructs the
runtime `RationalApproxData` realization before accepting the file.

## Case References

Case files reference sidecars from component parameters:

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

Relative `fit_file` paths are resolved against the directory containing the
case file. Absolute paths are accepted for local studies. If `sha256` is
present, it must be a 64-character hexadecimal SHA-256 of the sidecar bytes and
is verified before JSON parsing.

## Metadata And Validation

`source`, `validation`, and `metadata` are objects. `source` and `validation`
should contain reproducibility and quality information when generated data is
checked in. `metadata` is intentionally open-ended for generator-specific
details such as OpenLine conductor geometry, operating frequency, and fitting
method settings.

## Version 1 Scope

Version 1 sidecars are complete for characteristic-admittance `Yc` injection.
They do not store propagation functions or delay/history data. EMT case JSON
for `BranchFrequencyDependent` rejects a `propagation` parameter until that
runtime design is implemented.

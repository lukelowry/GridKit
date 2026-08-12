# Exact gates for the piecewise model

Two stages. Stage 1 is the deliverable for the study: `Math::sigmoid`'s
Piecewise form becomes the exact unit step, making the piecewise mode the
classical switched model with no gate parameter, in one primitive change.
Stage 2 (integrator event location) is the escalation if the exact model
proves non-integrable at scale, and reuses Stage 1 unchanged.

**Status: Stage 1 is implemented** (uncommitted). Two findings from the
implementation that amend the design below:

1. The primal-only comparison cannot appear inline in the residual after
   all: the select's i1 enters Enzyme's auto-sparsity value analysis and
   fails it ("not sparse solvable" / "cannot tell if depends on loop iv"),
   and a noinline kernel does not shield it. The landed form carries the
   jump with `ceil`, whose derivative both AD paths treat as exactly zero:
   `H(x) = ceil(r / (1 + r))`, `r = fmax(x, 0)` -- exact 0 for x <= 0,
   exact 1 for any positive x, denominator >= 1 so no singularity, and the
   IR is pure float arithmetic. The DependencyTracking instantiation keeps
   the primal comparison and returns a dependency-free constant.
2. "A gate contributes no Jacobian entries through its argument" is
   structural and shrinks patterns: REGCA's (IP, IL) LVPL-ceiling entry
   exists only through the smooth gate's transition tail, so the exact
   model legitimately drops it (a pinned Ip follows the ceiling's rate,
   not its position). The REGCA unit test's structural guard is now
   Smooth-mode-only; the DT-vs-Enzyme row equality runs in both modes.
   Suite: 66/66 green.

## Stage 1: exact heaviside gate (recommended, minimal)

### The observation

The ramp family is exact in Piecewise mode because max(x, 0) is an fmax
composition. The gate cannot be: fmax compositions are continuous, and the
unit step is not. A discontinuity requires a primal-only comparison - and
that is not a compromise, it is the classical semantics. The step's
derivative is structurally zero on both branches, which is exactly the
frozen-branch Jacobian of the classical model, and a comparison whose
result selects between constants has an identically zero tangent. This is
how ANDES evaluates its discrete flags: comparisons on the current
iterate, re-checked at every equation evaluation, flags entering the
piecewise equations as 0/1 factors, no event detection.

### The primitive

```cpp
/**
 * @brief Exact unit step, the gate of the classical switched model
 *
 * Deliberately not an fmax composition (fmax compositions are continuous;
 * a jump cannot be built from them). The comparison is primal-only: the
 * derivative of the step is structurally zero on both branches, the
 * frozen-branch Jacobian of the classical model. Enzyme's tangent of a
 * constant-armed select is identically zero; the DependencyTracking path
 * compares primal values through the existing Variable relational
 * operators and returns a dependency-free constant. Both AD paths agree
 * that gate terms contribute no Jacobian entries through their argument.
 *
 * H(0) = 0 by the strict comparison; exact initialization is strictly
 * interior and never evaluates on the boundary.
 */
template <class ScalarT>
__attribute__((always_inline)) inline ScalarT heaviside(const ScalarT x)
{
  using RealT = typename GridKit::ScalarTraits<ScalarT>::RealT;
  return x > ScalarT{ZERO<RealT>} ? ScalarT{ONE<RealT>} : ScalarT{ZERO<RealT>};
}
```

One template covers every scalar path: plain double, both Enzyme
instantiations (double code), and DependencyTracking's Variable, whose
relational operators already compare primal values and whose constant
construction carries no dependencies.

`sigmoid` delegates and the PWL step is deleted:

```cpp
template <Smoothing M = Smoothing::Smooth, class ScalarT>
__attribute__((always_inline)) inline ScalarT sigmoid(const ScalarT x)
{
  using RealT = typename GridKit::ScalarTraits<ScalarT>::RealT;
  if constexpr (M == Smoothing::Piecewise)
  {
    return heaviside(x);
  }
  else
  {
    return HALF<RealT> * (ONE<RealT> + std::tanh(HALF<RealT> * MU<RealT> * x));
  }
}
```

### Target usage

None of the models change. Every gate composition - `above`, `below`,
`inside`, `outside`, `indicator`, `antiwindup`, `deadband1` - is built on
`sigmoid<M>` and becomes exact automatically, the same way the ramp
family's exactness propagates from `ramp`. The anti-windup indicator
evaluates to exactly 0 or 1, so a state row reads

    r = x' - f            interior      (indicator = 1)
    r = x'                blocked       (indicator = 0, dynamics frozen)

which is the classical conditional-integration rule, evaluated per
residual call on the current iterate - ANDES's flag semantics under an
adaptive-step integrator. IDA's corrector and error test see the true
jump; steps shrink to localize each crossing to integration tolerance
(better than ANDES's fixed-step O(h) crossing error) and grow again after.

### What is deleted

- `GATE_MU` (Smoothing.hpp) and its doc block;
- the PWL step body and its fmax-difference note in `sigmoid`;
- `math.gate_mu` parsing and the GRIDKIT_SYSTEM echo field;
- the study harness `gate_mu` plumbing.

`PW_UNIT` stays (it serves the ramp family's tangent symmetry breaking).
Net diff is negative. The piecewise mode has no sharpness parameter left
anywhere: ramp/qramp exact by fmax, gates exact by heaviside.

### AD and sparsity

- Enzyme: the select's arms are constants, so the tangent is identically
  zero; the gate argument is primal-only. If the auto-sparsity pass
  objects to the select, the one-line fallback is a noinline wrapper
  registered inactive with Enzyme - identical semantics.
- DependencyTracking: the product term `indicator * f` keeps its
  structural dependence on f (dependency sets do not drop on zero
  values), and the gate argument contributes none. The pattern is fixed
  across branches - a subset of the old PWL pattern (the gate's own
  argument column disappears) - so the KLU symbolic factorization is
  stable; entry values flip 0/1 as branches switch.
- Both AD paths agree at interior points bit for bit with the old PWL
  gate: the step and the PWL gate are equal outside the PWL transition
  band of width 1/gate_mu, so golden residual vectors and exact
  initialization (strictly interior) are unchanged.

### Known deviations, by design

- Anti-windup freezes a state where the localized crossing left it,
  within integration tolerance past the limit; there is no exact snap to
  the limit value. Stage 2 snaps exactly.
- A tangential (grazing) crossing can chatter the step size toward h_min.
  At scale this is not a defect of the implementation, it is the study's
  finding-3 measurement on the true model: whether the exact switched
  model is integrable through a fault by step control alone.

### Validation

1. Suite green in Smooth mode (untouched) and Piecewise goldens
   unchanged (interior evaluation points).
2. TwoBusTgov1: limiter engage/release times against the analytic
   crossing; trajectory is the gate_mu -> infinity limit of the retired
   PWL runs (the mu-sweep history provides the sequence).
3. ThreeBusBasic control: bit-identical counters in both modes.
4. ACTIVSg10k piecewise rerun: the real test of integrability without
   event location.

## Stage 2: event location (escalation, unchanged from prior draft)

If Stage 1 stalls at h_min on the large cases, add integrator event
handling on top of it: mode bits per gate frozen during a step,
switching functions from the same expressions handed to IDARootInit,
and on IDA_ROOT_RETURN flip bits, snap pinned states exactly to their
limits, consistent-reinit (IDACalcIC) and IDAReInit at the crossing.
Stage 1's heaviside residual evaluated with frozen bits is exactly the
per-branch residual Stage 2 integrates, so nothing in Stage 1 is
throwaway. The earlier revision of this document holds the full Stage 2
design (interfaces, root routing, guards, counters); it is deliberately
not built until Stage 1's measurements demand it.

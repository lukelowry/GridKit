#pragma once

namespace GridKit
{
  namespace Math
  {
    /**
     * @brief Functional form used by the CommonMath primitives
     *
     * Selected per primitive instantiation through the leading template
     * parameter. Smooth is the default everywhere, so existing call sites
     * are unchanged; residuals that support both forms thread the parameter
     * through and dispatch on @ref SMOOTHING_MODE outside the differentiated
     * code path.
     *
     * Piecewise composes every primitive from std::fmax alone. That is
     * deliberate: fmax is the one kink-producing operation with a matching
     * DependencyTracking derivative rule and a validated Enzyme lowering
     * (see the notes at CommonMath's qramp). Do not introduce comparisons,
     * selects, or abs-based groupings when extending this family.
     *
     * A third, event-driven hard-switching mode is deliberately not defined
     * here. It requires integrator event detection and belongs to future
     * work; reserve the next enumerator for it.
     */
    enum class Smoothing
    {
      Smooth,   ///< C-infinity tanh/softplus forms
      Piecewise ///< exact fmax compositions; step becomes piecewise-linear
    };

    /**
     * @brief Runtime-selected smoothing mode for code outside residuals
     *
     * Residual and Jacobian entry points read this once per evaluation to
     * pick a primitive instantiation; initialization helpers read it to
     * invert the matching functional form. Set it before a system model is
     * constructed and do not change it mid-simulation.
     */
    inline Smoothing SMOOTHING_MODE = Smoothing::Smooth;

    /**
     * @brief Smoothing scale shared by CommonMath primitives
     *
     * Used by CommonMath's sigmoid, ramp, and functions composed from them
     * to set the width of smooth transitions. In the Piecewise mode the same
     * value sets the slope of the piecewise-linear unit step, so both
     * families share one sharpness scale.
     *
     * Runtime-assignable so parameter sweeps do not require a rebuild. Set
     * it together with @ref SMOOTHING_MODE before a system model is
     * constructed; exact-initialization inverses bake the active value into
     * the initial state.
     *
     * @tparam RealT - real data type
     */
    template <typename RealT>
    inline RealT MU = 240.0;

    /**
     * @brief Slope of the piecewise-linear unit step
     *
     * The Piecewise family's ramp compositions are exact and mu independent;
     * only its step gates carry a sharpness, and it is this one. Keeping it
     * separate from @ref MU lets a study sweep the smooth family's mu while
     * the piecewise arm stays one fixed reference model. Defaults to the
     * same value as MU; set both together before a system model is
     * constructed.
     *
     * @tparam RealT - real data type
     */
    template <typename RealT>
    inline RealT GATE_MU = 240.0;

    /**
     * @brief Runtime unit factor for piecewise tangent symmetry breaking
     *
     * Always exactly 1.0; multiplying by it never changes a value. Piecewise
     * forms whose tangent would otherwise carry several identical literal
     * unit coefficients in one residual row multiply one term by this
     * factor. LLVM fuses same-literal select coefficients into i1 arithmetic
     * that Enzyme's auto-sparsity solver rejects as not sparse solvable; a
     * runtime load keeps the coefficient symbolic. Never assign to it.
     *
     * @tparam RealT - real data type
     */
    template <typename RealT>
    inline RealT PW_UNIT = 1.0;
  } // namespace Math
} // namespace GridKit

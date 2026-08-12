#pragma once

#include <cassert>
#include <cmath>
#include <type_traits>

#include <GridKit/Constants.hpp>
#include <GridKit/ScalarTraits.hpp>
#include <GridKit/Smoothing.hpp>

namespace GridKit
{
  namespace Math
  {

    /**
     * @brief Exact unit step, the gate of the classical switched model
     *
     * Computed as ceil(r / (1 + r)) with r = fmax(x, 0): exactly 0 for
     * x <= 0 and exactly 1 for any positive x (r/(1+r) lies in (0, 1) and
     * ceil rounds it up; the denominator is at least 1, so there is no
     * singularity).
     *
     * @note A jump cannot be an fmax composition (those are continuous),
     * and writing the step as a comparison-and-select defeats Enzyme's
     * auto-sparsity solver (the i1 enters the value graph: "not sparse
     * solvable"). ceil is the one jump-carrying operation whose derivative
     * both AD paths agree is exactly zero, which is also the
     * frozen-branch Jacobian of the classical model: a gate contributes
     * no Jacobian entries through its argument. The DependencyTracking
     * instantiation uses the primal comparison directly and returns a
     * dependency-free constant.
     *
     * @note H(0) = 0. Exact initialization is strictly interior and never
     * evaluates on the boundary.
     *
     * @tparam ScalarT - scalar data type
     *
     * @param[in] x - input signal
     * @return exactly 1 for x > 0, exactly 0 otherwise
     */
    template <class ScalarT>
    __attribute__((always_inline)) inline ScalarT heaviside(const ScalarT x)
    {
      using RealT = typename GridKit::ScalarTraits<ScalarT>::RealT;
      if constexpr (std::is_same_v<ScalarT, RealT>)
      {
        const ScalarT r = std::fmax(x, ScalarT{ZERO<RealT>});
        return std::ceil(r / (ONE<RealT> + r));
      }
      else
      {
        return x > ScalarT{ZERO<RealT>} ? ScalarT{ONE<RealT>} : ScalarT{ZERO<RealT>};
      }
    }

    /**
     * @brief Scaled sigmoid activation function
     *
     * @note The sigmoid constant (mu) value is chosen to balance accuracy
     * and finite derivatives. Large values more closely approximate a step
     * function, but can make the transition numerically stiff.
     *
     * @note The Piecewise form is the exact unit step @ref heaviside, so
     * every gate composed from this function switches discontinuously at
     * its boundary, re-evaluated from the current iterate on every
     * residual and Jacobian call. Crossings are resolved by the
     * integrator's step control, not by event location.
     *
     * @tparam M - smoothing mode
     * @tparam ScalarT - scalar data type
     *
     * @param[in] x - expected to be of order 1
     * @return value of the sigmoid function
     */
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

    /**
     * @brief One-sided ramp function
     *
     * Smooth approximation to max(x, 0), using a stable softplus form with
     * the same scale as the rest of CommonMath.
     *
     * @note The Piecewise form is exact: fmax(x, 0), independent of mu.
     * Every primitive composed from ramp (max, min, clamp, deadband2, slew,
     * linseg) therefore becomes its exact piecewise counterpart under that
     * mode. Written with fmax, never abs; see the notes at @ref qramp.
     *
     * @tparam M - smoothing mode
     * @tparam ScalarT - scalar data type
     *
     * @param[in] x - expected to be of order 1
     * @return value of the ramp function
     */
    template <Smoothing M = Smoothing::Smooth, class ScalarT>
    __attribute__((always_inline)) inline ScalarT ramp(const ScalarT x)
    {
      using RealT = typename GridKit::ScalarTraits<ScalarT>::RealT;

      if constexpr (M == Smoothing::Piecewise)
      {
        // Deliberately unscaled: compositions that pair this ramp with a
        // passthrough term scale the passthrough by PW_UNIT instead, so the
        // two tangent coefficients stay distinct and cannot be re-factored
        // into the fused select form Enzyme's sparsity solver rejects.
        return std::fmax(x, ScalarT{ZERO<RealT>});
      }
      else
      {
        RealT   mu = MU<RealT>;
        ScalarT a  = std::abs(mu * x);
        return HALF<RealT> * (x + a / mu) + std::log1p(std::exp(-a)) / mu;
      }
    }

    /**
     * @brief Smooth one-sided quadratic ramp
     *
     * Smooth approximation to max(x, 0)^2, used for IEEE-style quadratic
     * saturation curves.
     *
     * ```
     * q(x) = x^2 * sigmoid(x)
     * ```
     *
     * @note Why not the exact piecewise definition? max(x, 0)^2 is already C^1
     * -- factored as x * max(x, 0), its derivative max(x, 0) + x H(x) multiplies
     * the undefined step H(0) by x = 0, so the derivative is uniquely zero at
     * the kink. Newton has everything it needs. The second derivative, however,
     * jumps from 0 to 2 there, and that jump is not free: on a 2000-bus case
     * where many machines sit near their saturation knee, the exact form cost
     * the BDF error estimator about 6% more steps and 19% more Jacobian
     * evaluations, for roughly 8% more wall time. Each individual evaluation
     * was cheaper; the integrator gave the saving back and more. The smooth
     * form keeps the second derivative continuous and is therefore faster
     * overall, at the price of being slightly unfaithful within a narrow band
     * around the knee. The exact form is kept, commented out, immediately below
     * this function as the baseline to compare against.
     *
     * @note The @c tanh gate is load-bearing and cheaper alternatives do not
     * substitute for it. Because @c tanh saturates to exactly +/-1 in double,
     * this expression returns exactly 0 below x = -0.159 and exactly x^2 above
     * x = 0.154, so away from a narrow band it is not an approximation at all.
     * Three things depend on that: GENROU and GENSAL initialize @c ksat with
     * this same primitive and rely on the residual matching bit for bit; the
     * golden residual vectors assume it; and both
     * autodiff paths agree on a hard zero rather than having to round a
     * negligible tail the same way. Algebraic smoothings have algebraic tails
     * and satisfy none of these -- the square of a hyperbola-smoothed ramp,
     * (x + sqrt(x^2 + eps^2))^2 / 4, is 4.6x cheaper at 1.65 ns against
     * 7.51 ns, monotone, nonnegative, and has curvature 2.00 against 2.32, yet
     * it still reads 1.9e-11 at x = -1, which is enough to break initialization
     * consistency and to make DependencyTracking report a 1.0e-12 partial that
     * Enzyme flushes to zero and drops. A softplus ramp squared decays faster
     * but likewise never reaches exact zero. Do not swap the gate without
     * re-checking those three invariants.
     *
     * @note The known defect of this form is that it is not monotone. Below the
     * knee it rises to a spurious peak of 8.4e-6 near x = -0.0092 before
     * falling again, so saturation is faintly nonzero with the wrong sign of
     * slope where it should be identically zero. The peak scales as 0.54 / mu^2
     * and the band where relative error exceeds 1% as 4.6 / mu, so both tighten
     * quickly if mu is raised. Unlike @ref ramp, curvature here is independent
     * of mu -- the x^2 prefactor cancels the sigmoid's spike, leaving max|q''|
     * at 2.32 for every mu tested from 240 to 1e6 -- so mu may be raised for
     * fidelity without introducing stiffness. That is untested against step
     * count and is the first thing to try if the knee accuracy matters.
     *
     * @note The Piecewise instantiation evaluates max(x, 0)^2 to the last
     * bit. It is the reference the smooth form is judged against: running
     * both isolates how much of a result depends on the smoothing rather
     * than on the model. Re-run the step-count comparison in the notes above
     * whenever the integrator, the tolerances, or the case mix change --
     * that trade was measured, not derived, and it will not hold forever.
     * The exact form is written with fmax, never with abs. Groupings built
     * from x + |x| put x d|x|/dx next to |x| dx in the tangent, which LLVM
     * fuses into a (1 + sign(x)) select; the resulting i1 either defeats
     * Enzyme's sparsity solver outright ("No sparsification: not sparse
     * solvable") or, in the groupings that do compile, silently yields a
     * wrong partial below the knee. fmax keeps Enzyme and DependencyTracking
     * in agreement, down to the structural zero at the knee -- which is why
     * DependencyTracking carries an fmax rule.
     *
     * @tparam M - smoothing mode
     * @tparam ScalarT - scalar data type
     *
     * @param[in] x - input signal
     * @return value of the quadratic ramp
     */
    template <Smoothing M = Smoothing::Smooth, class ScalarT>
    __attribute__((always_inline)) inline ScalarT qramp(const ScalarT x)
    {
      if constexpr (M == Smoothing::Piecewise)
      {
        using RealT     = typename GridKit::ScalarTraits<ScalarT>::RealT;
        const ScalarT r = std::fmax(x, ScalarT{ZERO<RealT>});
        return r * r;
      }
      else
      {
        return x * x * sigmoid<M>(x);
      }
    }

    /**
     * @brief Smooth binary maximum function
     *
     * Smooth approximation to max(x, y), composed from the smooth ramp
     * function.
     *
     * @tparam LeftT - scalar type of x
     * @tparam RightT - scalar type of y
     *
     * @param[in] x - First input signal
     * @param[in] y - Second input signal
     * @return Smooth maximum of x and y
     *
     * @note The two input types intentionally may differ. Model equations
     * often compare a differentiable state or signal with a plain real
     * parameter, limit, or literal bound. Keeping both template parameters
     * lets the expression promote to the differentiable scalar type without
     * forcing callers to cast every parameter.
     */
    template <Smoothing M = Smoothing::Smooth, class LeftT, class RightT>
    __attribute__((always_inline)) inline auto max(
        const LeftT  x,
        const RightT y)
    {
      if constexpr (M == Smoothing::Piecewise)
      {
        // Direct fmax keeps the tangent a bare select, the one kink shape
        // Enzyme's auto-sparsity pass lowers correctly; recombining ramp
        // selects with passthrough terms is rejected or silently
        // miscompiled. Promotion mirrors the smooth composition's type.
        using PromT = decltype(y + ramp<M>(x - y));
        return std::fmax(PromT{x}, PromT{y});
      }
      else
      {
        return y + ramp<M>(x - y);
      }
    }

    /**
     * @brief Smooth binary minimum function
     *
     * Smooth approximation to min(x, y), composed from the smooth ramp
     * function.
     *
     * @tparam LeftT - scalar type of x
     * @tparam RightT - scalar type of y
     *
     * @param[in] x - First input signal
     * @param[in] y - Second input signal
     * @return Smooth minimum of x and y
     *
     * @note The two input types intentionally may differ. Model equations
     * often compare a differentiable state or signal with a plain real
     * parameter, limit, or literal bound. Keeping both template parameters
     * lets the expression promote to the differentiable scalar type without
     * forcing callers to cast every parameter.
     */
    template <Smoothing M = Smoothing::Smooth, class LeftT, class RightT>
    __attribute__((always_inline)) inline auto min(
        const LeftT  x,
        const RightT y)
    {
      if constexpr (M == Smoothing::Piecewise)
      {
        // See max: direct fmax on negated arguments is the exact minimum
        // with a bare-select tangent; IEEE negation keeps it bit-exact.
        using PromT = decltype(x - ramp<M>(x - y));
        return -std::fmax(-PromT{x}, -PromT{y});
      }
      else
      {
        return x - ramp<M>(x - y);
      }
    }

    /**
     * @brief Smooth clamp function
     *
     * Smooth approximation to min(max(x, lower), upper), composed from the
     * smooth ramp function. Lower and upper bounds may be independent types
     * (e.g. constant Real bounds or algebraic-variable bounds).
     *
     * @tparam ScalarT - scalar data type of the input signal
     * @tparam LowerT - data type of the lower bound
     * @tparam UpperT - data type of the upper bound
     *
     * @param[in] x - expected to be of order 1
     * @param[in] lower - Lower limit
     * @param[in] upper - Upper limit
     * @return value of the smooth clamp function
     */
    template <Smoothing M = Smoothing::Smooth, class ScalarT, typename LowerT, typename UpperT>
    __attribute__((always_inline)) inline auto clamp(
        const ScalarT x,
        const LowerT  lower,
        const UpperT  upper)
    {
      assert(lower <= upper);
      // The ramp composition is exact in the Piecewise mode and its
      // difference-form tangent is validated through both AD paths; do not
      // rewrite it with nested fmax, which Enzyme's sparsity solver rejects.
      return lower + ramp<M>(x - lower) - ramp<M>(x - upper);
    }

    /**
     * @brief Smooth Type 1 no-offset two-sided deadband function
     *
     * Smooth approximation to a deadband that returns zero inside the band and
     * passes the input through unchanged outside the band.
     *
     * @tparam ScalarT - scalar data type
     * @tparam RealT - Real data type (see GridKit::ScalarTraits<ScalarT>::RealT)
     *
     * @param[in] x - Input signal
     * @param[in] lower - Lower breakpoint
     * @param[in] upper - Upper breakpoint
     * @return Smooth no-offset deadbanded value
     */
    template <Smoothing M = Smoothing::Smooth, class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT deadband1(
        const ScalarT x,
        const RealT   lower,
        const RealT   upper)
    {
      assert(lower <= upper);
      return x * (sigmoid<M>(lower - x) + sigmoid<M>(x - upper));
    }

    /**
     * @brief Smooth Type 2 offset two-sided deadband function
     *
     * Smooth approximation to x - min(max(x, lower), upper), composed from the
     * smooth ramp function.
     *
     * @tparam ScalarT - scalar data type
     * @tparam RealT - Real data type (see GridKit::ScalarTraits<ScalarT>::RealT)
     *
     * @param[in] x - Input signal
     * @param[in] lower - Lower breakpoint
     * @param[in] upper - Upper breakpoint
     * @return Smooth offset deadbanded value
     */
    template <Smoothing M = Smoothing::Smooth, class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT deadband2(
        const ScalarT x,
        const RealT   lower,
        const RealT   upper)
    {
      assert(lower <= upper);
      if constexpr (M == Smoothing::Piecewise)
      {
        // The exact clamp complement with a PW_UNIT-scaled passthrough. The
        // direct two-sided ramp difference sums same-coefficient selects and
        // the literal-unit complement fuses with them; both defeat Enzyme's
        // sparsity solver. PW_UNIT is exactly one, so values are exact.
        using RealTLocal = typename GridKit::ScalarTraits<ScalarT>::RealT;
        return PW_UNIT<RealTLocal> * x - clamp<M>(x, lower, upper);
      }
      else
      {
        return ramp<M>(x - upper) - ramp<M>(-(x - lower));
      }
    }

    /**
     * @brief Smooth slew-rate limiter
     *
     * Smooth approximation to min(max(f, -rate), rate).
     *
     * @tparam ScalarT - scalar data type
     * @tparam RealT - Real data type (see GridKit::ScalarTraits<ScalarT>::RealT)
     *
     * @param[in] f - Pre-limit derivative or rate signal
     * @param[in] rate - Symmetric positive rate limit
     * @return Slew-rate-limited value of f
     */
    template <Smoothing M = Smoothing::Smooth, class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT slew(
        const ScalarT f,
        const RealT   rate)
    {
      assert(rate >= ZERO<RealT>);
      return clamp<M>(f, -rate, rate);
    }

    /**
     * @brief Smooth linear segment contribution
     *
     * Smooth approximation to a linear segment contribution that is zero below
     * lower, linear over [lower, upper], and saturated at height above upper.
     * Callers should supply lower < upper; height may be positive or negative.
     *
     * @tparam ScalarT - scalar data type
     * @tparam RealT - Real data type (see GridKit::ScalarTraits<ScalarT>::RealT)
     *
     * @param[in] x - Input signal
     * @param[in] lower - Lower breakpoint
     * @param[in] upper - Upper breakpoint
     * @param[in] height - Saturated value above the upper breakpoint
     * @return Smooth linear segment contribution
     */
    template <Smoothing M = Smoothing::Smooth, class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT linseg(
        const ScalarT x,
        const RealT   lower,
        const RealT   upper,
        const RealT   height)
    {
      assert(lower < upper);
      return height / (upper - lower) * (ramp<M>(x - lower) - ramp<M>(x - upper));
    }

    /**
     * @brief Smooth above-limit indicator
     *
     * @tparam ScalarT - Scalar data type
     * @tparam RealT - Real data type (see GridKit::ScalarTraits<ScalarT>::RealT)
     *
     * @param[in] x - State variable
     * @param[in] limit_min - Minimum limit
     * @return Smooth indicator that x is above limit_min
     */
    template <Smoothing M = Smoothing::Smooth, class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT above(
        const ScalarT x,
        const RealT   limit_min)
    {
      return sigmoid<M>(x - limit_min);
    }

    /**
     * @brief Smooth below-limit indicator
     *
     * @tparam ScalarT - Scalar data type
     * @tparam RealT - Real data type (see GridKit::ScalarTraits<ScalarT>::RealT)
     *
     * @param[in] x - State variable
     * @param[in] limit_max - Maximum limit
     * @return Smooth indicator that x is below limit_max
     */
    template <Smoothing M = Smoothing::Smooth, class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT below(
        const ScalarT x,
        const RealT   limit_max)
    {
      return sigmoid<M>(limit_max - x);
    }

    /**
     * @brief Smooth inside-limits indicator
     *
     * @tparam ScalarT - Scalar data type
     * @tparam RealT - Real data type (see GridKit::ScalarTraits<ScalarT>::RealT)
     *
     * @param[in] x - State variable
     * @param[in] limit_min - Minimum limit
     * @param[in] limit_max - Maximum limit
     * @return Smooth indicator that x is inside [limit_min, limit_max]
     */
    template <Smoothing M = Smoothing::Smooth, class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT inside(
        const ScalarT x,
        const RealT   limit_min,
        const RealT   limit_max)
    {
      assert(limit_min <= limit_max);
      return above<M>(x, limit_min) + below<M>(x, limit_max) - ONE<RealT>;
    }

    /**
     * @brief Smooth outside-limits indicator
     *
     * @tparam ScalarT - Scalar data type
     * @tparam RealT - Real data type (see GridKit::ScalarTraits<ScalarT>::RealT)
     *
     * @param[in] x - State variable
     * @param[in] limit_min - Minimum limit
     * @param[in] limit_max - Maximum limit
     * @return Smooth indicator that x is outside [limit_min, limit_max]
     */
    template <Smoothing M = Smoothing::Smooth, class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT outside(
        const ScalarT x,
        const RealT   limit_min,
        const RealT   limit_max)
    {
      assert(limit_min <= limit_max);
      return below<M>(x, limit_min) + above<M>(x, limit_max);
    }

    /**
     * @brief Smooth anti-windup indicator for a limited state variable
     *
     * @tparam ScalarT - Scalar data type
     * @tparam RealT - Real data type (see GridKit::ScalarTraits<ScalarT>::RealT)
     *
     * @param[in] x - State variable
     * @param[in] f - Pre-limit derivative of the state variable
     * @param[in] limit_min - Minimum limit
     * @param[in] limit_max - Maximum limit
     * @return Scalar value in [0, 1]: 1 when dynamics should pass through,
     *         0 when integration should be blocked.
     */
    template <Smoothing M = Smoothing::Smooth, class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT indicator(
        const ScalarT x,
        const ScalarT f,
        const RealT   limit_min,
        const RealT   limit_max)
    {
      assert(limit_min <= limit_max);

      ScalarT above_min = above<M>(x, limit_min);
      ScalarT below_max = below<M>(x, limit_max);

      return above_min * below_max +                     //
             (ONE<RealT> - below_max) * sigmoid<M>(-f) + //
             (ONE<RealT> - above_min) * sigmoid<M>(f);
    }

    /**
     * @brief Smooth anti-windup limited derivative
     *
     * Applies the smooth anti-windup indicator gate to a pre-limit derivative.
     * The returned value approximates the conditional-integration rule that
     * passes interior dynamics, passes restoring motion from saturated limits,
     * and blocks motion that would push further into saturation.
     *
     * @tparam ScalarT - Scalar data type
     * @tparam RealT - Real data type (see GridKit::ScalarTraits<ScalarT>::RealT)
     *
     * @param[in] x - Limited state or limited output signal
     * @param[in] f - Pre-limit derivative
     * @param[in] limit_min - Minimum limit
     * @param[in] limit_max - Maximum limit
     * @return Smooth anti-windup limited derivative
     */
    template <Smoothing M = Smoothing::Smooth, class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT antiwindup(
        const ScalarT x,
        const ScalarT f,
        const RealT   limit_min,
        const RealT   limit_max)
    {
      return indicator<M>(x, f, limit_min, limit_max) * f;
    }
  } // namespace Math
} // namespace GridKit

#pragma once

#include <cassert>
#include <cmath>
#include <type_traits>

#include <GridKit/Constants.hpp>
#include <GridKit/ScalarTraits.hpp>

namespace GridKit
{
  namespace Math
  {
    template <typename RealT>
    inline constexpr RealT DEFAULT_MU = 240.0;

    template <typename RealT>
    inline constexpr RealT MU = DEFAULT_MU<RealT>;

    /**
     * @brief Scaled sigmoid activation function
     *
     * @note The sigmoid constant (mu) value is chosen to balance accuracy
     * and finite derivatives. Large values more closely approximate a step
     * function, but can make the transition numerically stiff.
     *
     * @tparam ScalarT - scalar data type
     *
     * @param[in] x - expected to be of order 1
     * @return value of the sigmoid function
     */
    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT sigmoid(const ScalarT x, const RealT mu)
    {
      return HALF<RealT> * (ONE<RealT> + std::tanh(HALF<RealT> * mu * x));
    }

    template <class ScalarT>
    __attribute__((always_inline)) inline ScalarT sigmoid(const ScalarT x)
    {
      using RealT = typename GridKit::ScalarTraits<ScalarT>::RealT;
      return sigmoid(x, DEFAULT_MU<RealT>);
    }

    /**
     * @brief Smooth one-sided ramp function
     *
     * Smooth approximation to max(x, 0), using a stable softplus form with
     * the same scale as the rest of CommonMath.
     *
     * @tparam ScalarT - scalar data type
     *
     * @param[in] x - expected to be of order 1
     * @return value of the smooth ramp function
     */
    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT ramp(const ScalarT x, const RealT mu)
    {
      ScalarT a = std::abs(mu * x);
      return HALF<RealT> * (x + a / mu) + std::log1p(std::exp(-a)) / mu;
    }

    template <class ScalarT>
    __attribute__((always_inline)) inline ScalarT ramp(const ScalarT x)
    {
      using RealT = typename GridKit::ScalarTraits<ScalarT>::RealT;
      return ramp(x, DEFAULT_MU<RealT>);
    }

    /**
     * @brief Smooth one-sided quadratic ramp
     *
     * Smooth approximation to max(x, 0)^2 via a sigmoid-gated quadratic.
     * Used for IEEE-style quadratic saturation curves.
     *
     * @todo Replace this smooth approximation with the exact piecewise
     *       definition since it is differentiable, just not twice
     *       differentiable.
     *
     * @tparam ScalarT - scalar data type
     *
     * @param[in] x - input signal
     * @return value of the quadratic ramp
     */
    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT qramp(const ScalarT x, const RealT mu)
    {
      return x * x * sigmoid(x, mu);
    }

    template <class ScalarT>
    __attribute__((always_inline)) inline ScalarT qramp(const ScalarT x)
    {
      using RealT = typename GridKit::ScalarTraits<ScalarT>::RealT;
      return qramp(x, DEFAULT_MU<RealT>);
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
    template <class LeftT, class RightT, typename RealT>
    __attribute__((always_inline)) inline auto max(
        const LeftT  x,
        const RightT y,
        const RealT  mu)
    {
      return y + ramp(x - y, mu);
    }

    template <class LeftT, class RightT>
    __attribute__((always_inline)) inline auto max(
        const LeftT  x,
        const RightT y)
    {
      using ScalarT = typename std::decay<decltype(x - y)>::type;
      using RealT   = typename GridKit::ScalarTraits<ScalarT>::RealT;
      return max(x, y, DEFAULT_MU<RealT>);
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
    template <class LeftT, class RightT, typename RealT>
    __attribute__((always_inline)) inline auto min(
        const LeftT  x,
        const RightT y,
        const RealT  mu)
    {
      return x - ramp(x - y, mu);
    }

    template <class LeftT, class RightT>
    __attribute__((always_inline)) inline auto min(
        const LeftT  x,
        const RightT y)
    {
      using ScalarT = typename std::decay<decltype(x - y)>::type;
      using RealT   = typename GridKit::ScalarTraits<ScalarT>::RealT;
      return min(x, y, DEFAULT_MU<RealT>);
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
    template <class ScalarT, typename LowerT, typename UpperT, typename RealT>
    __attribute__((always_inline)) inline auto clamp(
        const ScalarT x,
        const LowerT  lower,
        const UpperT  upper,
        const RealT   mu)
    {
      assert(lower <= upper);
      return lower + ramp(x - lower, mu) - ramp(x - upper, mu);
    }

    template <class ScalarT, typename LowerT, typename UpperT>
    __attribute__((always_inline)) inline auto clamp(
        const ScalarT x,
        const LowerT  lower,
        const UpperT  upper)
    {
      using DiffT = typename std::decay<decltype(x - lower)>::type;
      using RealT = typename GridKit::ScalarTraits<DiffT>::RealT;
      return clamp(x, lower, upper, DEFAULT_MU<RealT>);
    }

    /**
     * @brief Smooth two-sided deadband function
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
     * @return Smooth deadbanded value
     */
    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT deadband(
        const ScalarT x,
        const RealT   lower,
        const RealT   upper,
        const RealT   mu)
    {
      assert(lower <= upper);
      return ramp(x - upper, mu) - ramp(-(x - lower), mu);
    }

    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT deadband(
        const ScalarT x,
        const RealT   lower,
        const RealT   upper)
    {
      return deadband(x, lower, upper, DEFAULT_MU<RealT>);
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
    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT slew(
        const ScalarT f,
        const RealT   rate,
        const RealT   mu)
    {
      assert(rate >= ZERO<RealT>);
      return clamp(f, -rate, rate, mu);
    }

    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT slew(
        const ScalarT f,
        const RealT   rate)
    {
      return slew(f, rate, DEFAULT_MU<RealT>);
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
    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT linseg(
        const ScalarT x,
        const RealT   lower,
        const RealT   upper,
        const RealT   height,
        const RealT   mu)
    {
      assert(lower < upper);
      return height / (upper - lower) * (ramp(x - lower, mu) - ramp(x - upper, mu));
    }

    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT linseg(
        const ScalarT x,
        const RealT   lower,
        const RealT   upper,
        const RealT   height)
    {
      return linseg(x, lower, upper, height, DEFAULT_MU<RealT>);
    }

    /**
     * @brief Derivative of the scaled sigmoid activation function
     *        (i.e., approximation to the delta dirac function)
     *
     * @tparam ScalarT - scalar data type
     *
     * @param[in] x - expected to be of order 1
     * @return value of the sigmoid function
     */
    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT dsigmoid(const ScalarT x, const RealT mu)
    {
      return mu * sigmoid(x, mu) * (ONE<RealT> - sigmoid(x, mu));
    }

    template <class ScalarT>
    __attribute__((always_inline)) inline ScalarT dsigmoid(const ScalarT x)
    {
      using RealT = typename GridKit::ScalarTraits<ScalarT>::RealT;
      return dsigmoid(x, DEFAULT_MU<RealT>);
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
    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT above(
        const ScalarT x,
        const RealT   limit_min,
        const RealT   mu)
    {
      return sigmoid(x - limit_min, mu);
    }

    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT above(
        const ScalarT x,
        const RealT   limit_min)
    {
      return above(x, limit_min, DEFAULT_MU<RealT>);
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
    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT below(
        const ScalarT x,
        const RealT   limit_max,
        const RealT   mu)
    {
      return sigmoid(limit_max - x, mu);
    }

    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT below(
        const ScalarT x,
        const RealT   limit_max)
    {
      return below(x, limit_max, DEFAULT_MU<RealT>);
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
    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT inside(
        const ScalarT x,
        const RealT   limit_min,
        const RealT   limit_max,
        const RealT   mu)
    {
      assert(limit_min <= limit_max);
      return above(x, limit_min, mu) + below(x, limit_max, mu) - ONE<RealT>;
    }

    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT inside(
        const ScalarT x,
        const RealT   limit_min,
        const RealT   limit_max)
    {
      return inside(x, limit_min, limit_max, DEFAULT_MU<RealT>);
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
    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT outside(
        const ScalarT x,
        const RealT   limit_min,
        const RealT   limit_max,
        const RealT   mu)
    {
      assert(limit_min <= limit_max);
      return below(x, limit_min, mu) + above(x, limit_max, mu);
    }

    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT outside(
        const ScalarT x,
        const RealT   limit_min,
        const RealT   limit_max)
    {
      return outside(x, limit_min, limit_max, DEFAULT_MU<RealT>);
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
    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT indicator(
        const ScalarT x,
        const ScalarT f,
        const RealT   limit_min,
        const RealT   limit_max,
        const RealT   mu)
    {
      assert(limit_min <= limit_max);

      ScalarT above_min = above(x, limit_min, mu);
      ScalarT below_max = below(x, limit_max, mu);

      return above_min * below_max +                      //
             (ONE<RealT> - below_max) * sigmoid(-f, mu) + //
             (ONE<RealT> - above_min) * sigmoid(f, mu);
    }

    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT indicator(
        const ScalarT x,
        const ScalarT f,
        const RealT   limit_min,
        const RealT   limit_max)
    {
      return indicator(x, f, limit_min, limit_max, DEFAULT_MU<RealT>);
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
    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT antiwindup(
        const ScalarT x,
        const ScalarT f,
        const RealT   limit_min,
        const RealT   limit_max,
        const RealT   mu)
    {
      return indicator(x, f, limit_min, limit_max, mu) * f;
    }

    template <class ScalarT, typename RealT>
    __attribute__((always_inline)) inline ScalarT antiwindup(
        const ScalarT x,
        const ScalarT f,
        const RealT   limit_min,
        const RealT   limit_max)
    {
      return antiwindup(x, f, limit_min, limit_max, DEFAULT_MU<RealT>);
    }
  } // namespace Math
} // namespace GridKit

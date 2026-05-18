#pragma once

#include <cmath>

#include <GridKit/CommonMath.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class ScalarT>
    class SmoothnessIndicatorTests
    {
    public:
      SmoothnessIndicatorTests()  = default;
      ~SmoothnessIndicatorTests() = default;

      TestOutcome clamp()
      {
        TestStatus success = true;

        const ScalarT lower = -0.25;
        const ScalarT upper = 0.75;

        success *= (Math::clamp(static_cast<ScalarT>(-1.0), lower, upper) < lower + static_cast<ScalarT>(0.01));
        success *= (Math::clamp(static_cast<ScalarT>(0.4), lower, upper) > lower);
        success *= (Math::clamp(static_cast<ScalarT>(0.4), lower, upper) < upper);
        success *= (Math::clamp(static_cast<ScalarT>(1.5), lower, upper) > upper - static_cast<ScalarT>(0.01));

        return success.report(__func__);
      }

      TestOutcome deadband()
      {
        TestStatus success = true;

        const ScalarT lower = -0.05;
        const ScalarT upper = 0.10;

        success *= (Math::deadband(static_cast<ScalarT>(-1.0), lower, upper) < static_cast<ScalarT>(-0.94));
        success *= (Math::deadband(static_cast<ScalarT>(-1.0), lower, upper) > static_cast<ScalarT>(-0.96));
        success *= (std::abs(Math::deadband(static_cast<ScalarT>(0.02), lower, upper)) < static_cast<ScalarT>(1.0e-8));
        success *= (Math::deadband(static_cast<ScalarT>(1.0), lower, upper) > static_cast<ScalarT>(0.89));
        success *= (Math::deadband(static_cast<ScalarT>(1.0), lower, upper) < static_cast<ScalarT>(0.91));

        const ScalarT lower_breakpoint = Math::deadband(lower, lower, upper);
        const ScalarT upper_breakpoint = Math::deadband(upper, lower, upper);

        success *= (lower_breakpoint < static_cast<ScalarT>(0.0));
        success *= (upper_breakpoint > static_cast<ScalarT>(0.0));
        success *= (std::abs(lower_breakpoint) < static_cast<ScalarT>(0.003));
        success *= (std::abs(upper_breakpoint) < static_cast<ScalarT>(0.003));

        const ScalarT x  = -0.4;
        success         *= (std::abs(Math::deadband(x, lower, upper)
                             - (x - Math::clamp(x, lower, upper)))
                    < static_cast<ScalarT>(1.0e-12));

        success *= std::isfinite(Math::deadband(static_cast<ScalarT>(4.0), lower, upper));
        success *= (Math::deadband(static_cast<ScalarT>(4.0), lower, upper) > static_cast<ScalarT>(3.89));
        success *= std::isfinite(Math::deadband(static_cast<ScalarT>(-4.0), lower, upper));
        success *= (Math::deadband(static_cast<ScalarT>(-4.0), lower, upper) < static_cast<ScalarT>(-3.94));

        const ScalarT point  = 0.25;
        success             *= (std::abs(Math::deadband(static_cast<ScalarT>(0.75), point, point) - static_cast<ScalarT>(0.5))
                    < static_cast<ScalarT>(1.0e-12));
        success             *= (std::abs(Math::deadband(static_cast<ScalarT>(-0.25), point, point) + static_cast<ScalarT>(0.5))
                    < static_cast<ScalarT>(1.0e-12));

        return success.report(__func__);
      }

      TestOutcome limitIndicators()
      {
        TestStatus success = true;

        const ScalarT limit_min = 0.0;
        const ScalarT limit_max = 3.0;

        success *= (Math::above(static_cast<ScalarT>(1.0), limit_min) > static_cast<ScalarT>(0.99));
        success *= (Math::above(static_cast<ScalarT>(-0.2), limit_min) < static_cast<ScalarT>(0.1));
        success *= (Math::below(static_cast<ScalarT>(1.0), limit_max) > static_cast<ScalarT>(0.99));
        success *= (Math::below(static_cast<ScalarT>(3.2), limit_max) < static_cast<ScalarT>(0.1));

        success *= (Math::inside(static_cast<ScalarT>(1.5), limit_min, limit_max) > static_cast<ScalarT>(0.99));
        success *= (Math::inside(static_cast<ScalarT>(-0.2), limit_min, limit_max) < static_cast<ScalarT>(0.1));
        success *= (Math::inside(static_cast<ScalarT>(3.2), limit_min, limit_max) < static_cast<ScalarT>(0.1));

        success *= (Math::outside(static_cast<ScalarT>(1.5), limit_min, limit_max) < static_cast<ScalarT>(0.1));
        success *= (Math::outside(static_cast<ScalarT>(-0.2), limit_min, limit_max) > static_cast<ScalarT>(0.9));
        success *= (Math::outside(static_cast<ScalarT>(3.2), limit_min, limit_max) > static_cast<ScalarT>(0.9));

        const ScalarT x  = static_cast<ScalarT>(1.5);
        success         *= (std::abs(Math::inside(x, limit_min, limit_max)
                             + Math::outside(x, limit_min, limit_max)
                             - static_cast<ScalarT>(1.0))
                    < static_cast<ScalarT>(1.0e-12));

        return success.report(__func__);
      }

      TestOutcome slew()
      {
        TestStatus success = true;

        const ScalarT rate = 0.5;

        success *= (Math::slew(static_cast<ScalarT>(2.0), rate) < static_cast<ScalarT>(0.51));
        success *= (Math::slew(static_cast<ScalarT>(-2.0), rate) > static_cast<ScalarT>(-0.51));
        success *= (Math::slew(static_cast<ScalarT>(0.2), rate) > static_cast<ScalarT>(0.19));
        success *= (Math::slew(static_cast<ScalarT>(-0.2), rate) < static_cast<ScalarT>(-0.19));

        return success.report(__func__);
      }

      TestOutcome linseg()
      {
        TestStatus success = true;

        success *= (Math::linseg(static_cast<ScalarT>(-1.0),
                                 static_cast<ScalarT>(0.0),
                                 static_cast<ScalarT>(2.0),
                                 static_cast<ScalarT>(4.0))
                    < static_cast<ScalarT>(0.01));
        success *= (Math::linseg(static_cast<ScalarT>(1.0),
                                 static_cast<ScalarT>(0.0),
                                 static_cast<ScalarT>(2.0),
                                 static_cast<ScalarT>(4.0))
                    > static_cast<ScalarT>(1.99));
        success *= (Math::linseg(static_cast<ScalarT>(1.0),
                                 static_cast<ScalarT>(0.0),
                                 static_cast<ScalarT>(2.0),
                                 static_cast<ScalarT>(4.0))
                    < static_cast<ScalarT>(2.01));
        success *= (Math::linseg(static_cast<ScalarT>(3.0),
                                 static_cast<ScalarT>(0.0),
                                 static_cast<ScalarT>(2.0),
                                 static_cast<ScalarT>(4.0))
                    > static_cast<ScalarT>(3.99));
        success *= (Math::linseg(static_cast<ScalarT>(1.0),
                                 static_cast<ScalarT>(0.0),
                                 static_cast<ScalarT>(2.0),
                                 static_cast<ScalarT>(-4.0))
                    < static_cast<ScalarT>(-1.99));
        success *= (Math::linseg(static_cast<ScalarT>(1.0),
                                 static_cast<ScalarT>(0.0),
                                 static_cast<ScalarT>(2.0),
                                 static_cast<ScalarT>(-4.0))
                    > static_cast<ScalarT>(-2.01));

        return success.report(__func__);
      }

      TestOutcome ramp()
      {
        TestStatus success = true;

        const ScalarT tau     = static_cast<ScalarT>(1.0 / 240.0);
        const ScalarT at_zero = tau * std::log(static_cast<ScalarT>(2.0));

        success *= (Math::ramp(static_cast<ScalarT>(1.0)) > static_cast<ScalarT>(0.99));
        success *= (Math::ramp(static_cast<ScalarT>(-1.0)) < static_cast<ScalarT>(0.01));
        success *= (std::abs(Math::ramp(static_cast<ScalarT>(0.0)) - at_zero) < static_cast<ScalarT>(1.0e-12));
        success *= (Math::ramp(static_cast<ScalarT>(-0.01)) > static_cast<ScalarT>(0.0));
        success *= (Math::ramp(static_cast<ScalarT>(0.01)) > Math::ramp(static_cast<ScalarT>(0.0)));
        success *= std::isfinite(Math::ramp(static_cast<ScalarT>(4.0)));
        success *= (Math::ramp(static_cast<ScalarT>(4.0)) > static_cast<ScalarT>(3.99));
        success *= std::isfinite(Math::ramp(static_cast<ScalarT>(-4.0)));
        success *= (Math::ramp(static_cast<ScalarT>(-4.0)) < static_cast<ScalarT>(1.0e-12));

        const ScalarT lower = -0.25;
        const ScalarT upper = 0.75;
        const ScalarT x     = 0.4;

        const ScalarT smooth_clip  = lower + Math::ramp(x - lower) - Math::ramp(x - upper);
        success                   *= (smooth_clip > lower);
        success                   *= (smooth_clip < upper);
        success                   *= std::isfinite(Math::clamp(static_cast<ScalarT>(4.0), lower, upper));
        success                   *= (Math::clamp(static_cast<ScalarT>(4.0), lower, upper) < upper + static_cast<ScalarT>(1.0e-12));
        success                   *= std::isfinite(Math::clamp(static_cast<ScalarT>(-4.0), lower, upper));
        success                   *= (Math::clamp(static_cast<ScalarT>(-4.0), lower, upper) > lower - static_cast<ScalarT>(1.0e-12));

        return success.report(__func__);
      }

      TestOutcome minMax()
      {
        TestStatus success = true;

        const ScalarT high = static_cast<ScalarT>(2.0);
        const ScalarT low  = static_cast<ScalarT>(-1.0);

        success *= (Math::max(high, low) > high - static_cast<ScalarT>(0.01));
        success *= (Math::max(high, low) < high + static_cast<ScalarT>(0.01));
        success *= (Math::max(low, high) > high - static_cast<ScalarT>(0.01));
        success *= (Math::max(low, high) < high + static_cast<ScalarT>(0.01));

        success *= (Math::min(high, low) > low - static_cast<ScalarT>(0.01));
        success *= (Math::min(high, low) < low + static_cast<ScalarT>(0.01));
        success *= (Math::min(low, high) > low - static_cast<ScalarT>(0.01));
        success *= (Math::min(low, high) < low + static_cast<ScalarT>(0.01));

        const auto lower_bounded  = Math::max(static_cast<ScalarT>(-1.0), 0.01);
        success                  *= (lower_bounded > static_cast<ScalarT>(0.009));
        success                  *= (lower_bounded < static_cast<ScalarT>(0.011));

        const ScalarT x  = static_cast<ScalarT>(0.4);
        const ScalarT y  = static_cast<ScalarT>(-0.7);
        success         *= (std::abs(Math::min(x, y) + Math::max(x, y) - (x + y))
                    < static_cast<ScalarT>(1.0e-12));

        const ScalarT point = static_cast<ScalarT>(0.25);
        const ScalarT bias  = std::log(static_cast<ScalarT>(2.0)) / static_cast<ScalarT>(240.0);

        success *= (std::abs(Math::max(point, point) - (point + bias))
                    < static_cast<ScalarT>(1.0e-12));
        success *= (std::abs(Math::min(point, point) - (point - bias))
                    < static_cast<ScalarT>(1.0e-12));

        return success.report(__func__);
      }

      TestOutcome antiWindupIndicator()
      {
        TestStatus success = true;

        const ScalarT limit_min = 0.0;
        const ScalarT limit_max = 3.0;

        // Inside the limits the indicator passes dynamics through, regardless
        // of the sign of f: value is close to 1.
        success *= (Math::indicator(static_cast<ScalarT>(1.5), static_cast<ScalarT>(0.01), limit_min, limit_max) > static_cast<ScalarT>(0.99));

        // Above the upper limit with f pushing further out: blocked (≈ 0).
        success *= (Math::indicator(static_cast<ScalarT>(3.2), static_cast<ScalarT>(0.01), limit_min, limit_max) < static_cast<ScalarT>(0.1));

        // Above the upper limit but f pulling back in: passed (≈ 1).
        success *= (Math::indicator(static_cast<ScalarT>(3.2), static_cast<ScalarT>(-0.01), limit_min, limit_max) > static_cast<ScalarT>(0.9));

        // Below the lower limit with f pushing further out: blocked (≈ 0).
        success *= (Math::indicator(static_cast<ScalarT>(-0.2), static_cast<ScalarT>(-0.01), limit_min, limit_max) < static_cast<ScalarT>(0.1));

        // Below the lower limit but f pulling back in: passed (≈ 1).
        success *= (Math::indicator(static_cast<ScalarT>(-0.2), static_cast<ScalarT>(0.01), limit_min, limit_max) > static_cast<ScalarT>(0.9));

        return success.report(__func__);
      }

      TestOutcome antiWindup()
      {
        TestStatus success = true;

        const ScalarT limit_min = 0.0;
        const ScalarT limit_max = 3.0;

        const ScalarT f_positive = 0.01;
        const ScalarT f_negative = -0.01;

        success *= (std::abs(Math::antiwindup(static_cast<ScalarT>(1.5), f_positive, limit_min, limit_max)
                             - Math::indicator(static_cast<ScalarT>(1.5), f_positive, limit_min, limit_max) * f_positive)
                    < static_cast<ScalarT>(1.0e-12));

        success *= (Math::antiwindup(static_cast<ScalarT>(3.2), f_positive, limit_min, limit_max) < static_cast<ScalarT>(0.001));
        success *= (Math::antiwindup(static_cast<ScalarT>(3.2), f_negative, limit_min, limit_max) < static_cast<ScalarT>(-0.009));
        success *= (Math::antiwindup(static_cast<ScalarT>(-0.2), f_negative, limit_min, limit_max) > static_cast<ScalarT>(-0.001));
        success *= (Math::antiwindup(static_cast<ScalarT>(-0.2), f_positive, limit_min, limit_max) > static_cast<ScalarT>(0.009));

        return success.report(__func__);
      }
    };

  } // namespace Testing
} // namespace GridKit

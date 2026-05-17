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

      TestOutcome rampsat()
      {
        TestStatus success = true;

        success *= (Math::rampsat(static_cast<ScalarT>(-1.0),
                                  static_cast<ScalarT>(0.0),
                                  static_cast<ScalarT>(2.0),
                                  static_cast<ScalarT>(4.0)) < static_cast<ScalarT>(0.01));
        success *= (Math::rampsat(static_cast<ScalarT>(1.0),
                                  static_cast<ScalarT>(0.0),
                                  static_cast<ScalarT>(2.0),
                                  static_cast<ScalarT>(4.0)) > static_cast<ScalarT>(1.99));
        success *= (Math::rampsat(static_cast<ScalarT>(1.0),
                                  static_cast<ScalarT>(0.0),
                                  static_cast<ScalarT>(2.0),
                                  static_cast<ScalarT>(4.0)) < static_cast<ScalarT>(2.01));
        success *= (Math::rampsat(static_cast<ScalarT>(3.0),
                                  static_cast<ScalarT>(0.0),
                                  static_cast<ScalarT>(2.0),
                                  static_cast<ScalarT>(4.0)) > static_cast<ScalarT>(3.99));

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

        const ScalarT lower = -0.25;
        const ScalarT upper = 0.75;
        const ScalarT x     = 0.4;

        const ScalarT smooth_clip = lower + Math::ramp(x - lower) - Math::ramp(x - upper);
        success *= (smooth_clip > lower);
        success *= (smooth_clip < upper);

        return success.report(__func__);
      }

      TestOutcome antiWindupIndicator()
      {
        TestStatus success = true;

        const ScalarT limit_min = 0.0;
        const ScalarT limit_max = 3.0;

        // Inside the limits the indicator passes dynamics through, regardless
        // of the sign of f: value is close to 1.
        success *= (Math::indicator(limit_min, limit_max, static_cast<ScalarT>(1.5), static_cast<ScalarT>(0.01)) > static_cast<ScalarT>(0.99));

        // Above the upper limit with f pushing further out: blocked (≈ 0).
        success *= (Math::indicator(limit_min, limit_max, static_cast<ScalarT>(3.2), static_cast<ScalarT>(0.01)) < static_cast<ScalarT>(0.1));

        // Above the upper limit but f pulling back in: passed (≈ 1).
        success *= (Math::indicator(limit_min, limit_max, static_cast<ScalarT>(3.2), static_cast<ScalarT>(-0.01)) > static_cast<ScalarT>(0.9));

        // Below the lower limit with f pushing further out: blocked (≈ 0).
        success *= (Math::indicator(limit_min, limit_max, static_cast<ScalarT>(-0.2), static_cast<ScalarT>(-0.01)) < static_cast<ScalarT>(0.1));

        // Below the lower limit but f pulling back in: passed (≈ 1).
        success *= (Math::indicator(limit_min, limit_max, static_cast<ScalarT>(-0.2), static_cast<ScalarT>(0.01)) > static_cast<ScalarT>(0.9));

        return success.report(__func__);
      }
    };

  } // namespace Testing
} // namespace GridKit

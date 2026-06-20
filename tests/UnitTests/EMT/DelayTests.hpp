#include <cmath>
#include <stdexcept>

#include <GridKit/Model/EMT/Operators/Shift/Delay/Delay.hpp>
#include <GridKit/Testing/TestHelpers.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class EMTDelayTests
    {
      using Delay = EMT::Delay<ScalarT, IdxT>;
      using RealT = typename Delay::RealT;

    public:
      TestOutcome constructorValidation()
      {
        TestStatus success = true;

        success *= throws<std::invalid_argument>([]()
                                                 { Delay delay(-1.0, 10.0, constantInput, zeroInputDerivative); });
        success *= throws<std::invalid_argument>([]()
                                                 { Delay delay(1.0, 0.0, constantInput, zeroInputDerivative); });
        success *= throws<std::invalid_argument>([]()
                                                 { Delay delay(1.0, 10.0, nullptr, zeroInputDerivative); });

        return success.report(__func__);
      }

      TestOutcome derivedParameters()
      {
        TestStatus success = true;

        Delay delay(0.25, 9.0, constantInput, zeroInputDerivative);
        success *= (delay.sectionCount() == 3);
        success *= isEqual(delay.sectionTime(), 0.25 / 3.0, 1.0e-14);
        success *= (delay.size() == 3);
        success *= (delay.nnz() == 5);

        return success.report(__func__);
      }

      TestOutcome initializationUsesInputHistory()
      {
        TestStatus success = true;

        Delay delay(0.3, 10.0, quadraticInput, quadraticInputDerivative);
        delay.updateTime(0.0, 0.0);
        delay.initialize();

        const RealT T = delay.sectionTime();
        for (IdxT i = 0; i < delay.sectionCount(); ++i)
        {
          const RealT history_time  = -static_cast<RealT>(i + 1) * T;
          success                  *= isEqual(delay.y()[static_cast<size_t>(i)], quadraticInput(history_time), 1.0e-14);
          success                  *= isEqual(delay.yp()[static_cast<size_t>(i)], quadraticInputDerivative(history_time), 1.0e-14);
        }
        success *= isEqual(delay.output(), quadraticInput(-0.3), 1.0e-14);

        return success.report(__func__);
      }

      TestOutcome customInitializationSetsEachSection()
      {
        TestStatus success = true;

        Delay delay(
            0.3,
            10.0,
            quadraticInput,
            quadraticInputDerivative,
            [](IdxT i, RealT t, RealT T, ScalarT& y, ScalarT& yp)
            {
              y  = 10.0 + static_cast<RealT>(i) + t;
              yp = 20.0 + static_cast<RealT>(i) + T;
            });

        delay.updateTime(2.0, 0.0);
        delay.initialize();

        const RealT T = delay.sectionTime();
        for (IdxT i = 0; i < delay.sectionCount(); ++i)
        {
          success *= isEqual(delay.y()[static_cast<size_t>(i)], 10.0 + static_cast<RealT>(i) + 2.0, 1.0e-14);
          success *= isEqual(delay.yp()[static_cast<size_t>(i)], 20.0 + static_cast<RealT>(i) + T, 1.0e-14);
        }

        return success.report(__func__);
      }

      TestOutcome residual()
      {
        TestStatus success = true;

        Delay delay(0.3, 10.0, linearInput, linearInputDerivative);
        delay.updateTime(2.0, 0.0);

        delay.y()  = {1.0, 0.75, 0.5};
        delay.yp() = {0.2, 0.4, 0.6};
        delay.evaluateResidual();

        const auto& f  = delay.getResidual();
        const RealT T  = delay.sectionTime();
        success       *= isEqual(f[0], -T * 0.2 - 1.0 + linearInput(2.0), 1.0e-14);
        success       *= isEqual(f[1], -T * 0.4 - 0.75 + 1.0, 1.0e-14);
        success       *= isEqual(f[2], -T * 0.6 - 0.5 + 0.75, 1.0e-14);

        return success.report(__func__);
      }

      TestOutcome jacobian()
      {
        TestStatus success = true;

        Delay delay(0.3, 10.0, constantInput, zeroInputDerivative);
        success *= checkJacobian(delay, 0.0);
        success *= checkJacobian(delay, 1.0);

        return success.report(__func__);
      }

    private:
      static ScalarT constantInput(RealT)
      {
        return 1.0;
      }

      static ScalarT zeroInputDerivative(RealT)
      {
        return 0.0;
      }

      static ScalarT linearInput(RealT t)
      {
        return 1.0 + 2.0 * t;
      }

      static ScalarT linearInputDerivative(RealT)
      {
        return 2.0;
      }

      static ScalarT quadraticInput(RealT t)
      {
        return 1.0 + 2.0 * t + 0.5 * t * t;
      }

      static ScalarT quadraticInputDerivative(RealT t)
      {
        return 2.0 + t;
      }

      bool checkJacobian(Delay& delay, RealT alpha)
      {
        delay.updateTime(0.0, alpha);
        delay.evaluateJacobian();

        const RealT T        = delay.sectionTime();
        const RealT diagonal = -1.0 - alpha * T;
        const auto* values   = delay.getCsrJacobian()->getValues();

        return isEqual(values[0], diagonal, 1.0e-14)
               && isEqual(values[1], 1.0, 1.0e-14)
               && isEqual(values[2], diagonal, 1.0e-14)
               && isEqual(values[3], 1.0, 1.0e-14)
               && isEqual(values[4], diagonal, 1.0e-14);
      }
    };
  } // namespace Testing
} // namespace GridKit

#include <GridKit/Model/PhasorDynamics/History.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    class HistoryTests
    {
    public:
      TestOutcome scalarLookup()
      {
        using HistoryT = PhasorDynamics::History<double, double>;

        TestStatus success = true;
        HistoryT   history(1.0, 0.25);

        double hmax = 0.0;
        history.setMaxStepSize(hmax);
        success *= isEqual(hmax, 0.25, 1.0e-14);

        auto linear = [](double lower, double upper, double theta)
        {
          return lower + theta * (upper - lower);
        };

        success *= isEqual(history.heldValue(0.0, 3.0), 3.0, 1.0e-14);
        success *= isEqual(history.interpolatedValue(0.0, 3.0, linear), 3.0, 1.0e-14);

        history.record(0.0, 0.0);
        history.record(0.5, 1.0);
        history.record(1.0, 2.0);

        success *= isEqual(history.heldValue(0.25, 3.0), 0.0, 1.0e-14);
        success *= isEqual(history.interpolatedValue(0.25, 3.0, linear), 0.5, 1.0e-14);
        success *= isEqual(history.interpolatedValue(0.75, 3.0, linear), 1.5, 1.0e-14);

        return success.report(__func__);
      }

      TestOutcome pruningAndRestart()
      {
        using HistoryT = PhasorDynamics::History<double, double>;

        TestStatus success = true;
        HistoryT   history(1.0);

        auto linear = [](double lower, double upper, double theta)
        {
          return lower + theta * (upper - lower);
        };

        history.record(0.0, 0.0);
        history.record(0.5, 1.0);
        history.record(1.0, 2.0);
        history.record(1.5, 3.0);

        success *= (history.samples().size() == 3);
        success *= isEqual(history.interpolatedValue(0.25, 10.0, linear), 0.5, 1.0e-14);

        history.record(1.5, 4.0);
        success *= (history.samples().size() == 3);
        success *= isEqual(history.interpolatedValue(1.5, 10.0, linear), 4.0, 1.0e-14);

        history.record(0.25, 8.0);
        success *= (history.samples().size() == 1);
        success *= isEqual(history.heldValue(0.25, 10.0), 8.0, 1.0e-14);

        return success.report(__func__);
      }
    };
  } // namespace Testing
} // namespace GridKit

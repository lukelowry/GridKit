#pragma once

#include <cmath>
#include <limits>

#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/EMT/Source/VoltageSource/VoltageSource.hpp>
#include <GridKit/Testing/TestHelpers.hpp>
#include <GridKit/Testing/Testing.hpp>
#include <GridKit/Utilities/MapFromCOO.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class VoltageSourceTests
    {
    public:
      using RealT       = typename GridKit::ScalarTraits<ScalarT>::RealT;
      using BusDataT    = EMT::BusData<RealT, IdxT>;
      using SourceDataT = EMT::VoltageSourceData<RealT, IdxT>;

      TestOutcome initialization()
      {
        TestStatus success = true;

        EMT::Bus<ScalarT, IdxT>           bus(makeBusData());
        EMT::VoltageSource<ScalarT, IdxT> source(makeSourceData());
        prepare(bus, source);

        source.initialize();
        source.tagDifferentiable();

        const auto omega = angularFrequency();
        const auto tol   = static_cast<RealT>(1.0e-12);

        success *= (source.size() == 9);
        success *= isEqual(source.y()[0], static_cast<ScalarT>(1.0), tol);
        success *= isEqual(source.y()[1], static_cast<ScalarT>(-2.0), tol);
        success *= isEqual(source.y()[2], static_cast<ScalarT>(3.0), tol);

        success *= isEqual(source.y()[3], static_cast<ScalarT>(0.0), tol);
        success *= isEqual(source.y()[4], static_cast<ScalarT>(-sqrt3()), tol);
        success *= isEqual(source.y()[5], static_cast<ScalarT>(sqrt3()), tol);

        success *= isEqual(source.y()[6], static_cast<ScalarT>(2.0 * omega), tol);
        success *= isEqual(source.y()[7], static_cast<ScalarT>(-omega), tol);
        success *= isEqual(source.y()[8], static_cast<ScalarT>(-omega), tol);

        for (size_t phase = 0; phase < 3; ++phase)
        {
          success *= !source.tag()[phase];
        }
        for (size_t state = 3; state < 9; ++state)
        {
          success *= source.tag()[state];
        }

        return success.report(__func__);
      }

      TestOutcome residual()
      {
        TestStatus success = true;

        EMT::Bus<ScalarT, IdxT>           bus(makeBusData());
        EMT::VoltageSource<ScalarT, IdxT> source(makeSourceData());
        prepare(bus, source);

        bus.initialize();
        source.initialize();
        source.evaluateResidual();

        const auto tol = static_cast<RealT>(1.0e-10);
        for (auto value : source.getResidual())
        {
          success *= isEqual(value, static_cast<ScalarT>(0.0), tol);
        }

        bus.y()[0] = 0.25;
        source.evaluateResidual();
        success *= isEqual(source.getResidual()[0], static_cast<ScalarT>(0.25), tol);

        return success.report(__func__);
      }

      TestOutcome jacobian()
      {
        TestStatus success = true;

        EMT::Bus<ScalarT, IdxT>           bus(makeBusData());
        EMT::VoltageSource<ScalarT, IdxT> source(makeSourceData());
        prepare(bus, source);

        constexpr RealT alpha  = 5.0;
        const RealT     omega2 = angularFrequency() * angularFrequency();

        source.updateTime(0.0, alpha);
        source.evaluateJacobian();

        auto                                                     jacobian = MapFromCOO<RealT, IdxT>(source.getJacobian());
        std::vector<DependencyTracking::Variable::DependencyMap> expected(12);

        for (IdxT phase = 0; phase < 3; ++phase)
        {
          const auto current    = static_cast<IdxT>(3) + phase;
          const auto voltage    = static_cast<IdxT>(6) + phase;
          const auto derivative = static_cast<IdxT>(9) + phase;

          expected[phase][current]         = -1.0;
          expected[current][phase]         = 1.0;
          expected[current][voltage]       = -1.0;
          expected[voltage][voltage]       = alpha;
          expected[voltage][derivative]    = -1.0;
          expected[derivative][voltage]    = omega2;
          expected[derivative][derivative] = alpha;
        }

        const auto tol = static_cast<RealT>(100.0) * std::numeric_limits<RealT>::epsilon();
        for (size_t row = 0; row < expected.size(); ++row)
        {
          success *= isEqual<IdxT, RealT>(jacobian[row], expected[row], tol);
        }
        success *= (source.nnz() == 21);

        return success.report(__func__);
      }

    private:
      static constexpr auto sqrt3() -> RealT
      {
        return static_cast<RealT>(1.732050807568877293527446341505872367);
      }

      static constexpr auto pi() -> RealT
      {
        return static_cast<RealT>(3.141592653589793238462643383279502884);
      }

      static auto angularFrequency() -> RealT
      {
        return static_cast<RealT>(2.0) * pi() * static_cast<RealT>(60.0);
      }

      static auto makeBusData() -> BusDataT
      {
        BusDataT data;
        data.v0 = {0.0, -sqrt3(), sqrt3()};
        return data;
      }

      static auto makeSourceData() -> SourceDataT
      {
        SourceDataT data;
        data.amplitude = 2.0;
        data.frequency = 60.0;
        data.current0  = {1.0, -2.0, 3.0};
        return data;
      }

      static void prepare(EMT::Bus<ScalarT, IdxT>&           bus,
                          EMT::VoltageSource<ScalarT, IdxT>& source)
      {
        bus.allocate();
        source.allocate();

        bus.setIndexRanges({0, 3}, {0, 3});
        source.setIndexRanges({3, 9}, {3, 9});

        typename EMT::Component<ScalarT, IdxT>::TerminalView terminal;
        terminal.y              = bus.y().data();
        terminal.yp             = bus.yp().data();
        terminal.variable_index = bus.variableIndex(0);
        terminal.residual_index = bus.residualIndex(0);
        source.bindTerminal(0, terminal);
      }
    };

  } // namespace Testing
} // namespace GridKit

#pragma once

#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/EMT/Load/ShuntLoad.hpp>
#include <GridKit/Testing/TestHelpers.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class ShuntLoadTests
    {
    public:
      using RealT     = typename GridKit::ScalarTraits<ScalarT>::RealT;
      using BusDataT  = EMT::BusData<RealT, IdxT>;
      using LoadDataT = EMT::ShuntLoadData<RealT, IdxT>;

      TestOutcome switching()
      {
        TestStatus success = true;

        EMT::Bus<ScalarT, IdxT>       bus(makeBusData());
        EMT::ShuntLoad<ScalarT, IdxT> load(makeLoadData(false));
        prepare(bus, load);

        bus.initialize();
        load.evaluateResidual();

        const auto  tol     = static_cast<RealT>(1.0e-12);
        const auto* current = load.terminalCurrent(0);

        success *= !load.isClosed();
        for (size_t phase = 0; phase < 3; ++phase)
        {
          success *= isEqual(current[phase], static_cast<ScalarT>(0.0), tol);
        }

        load.setClosed(true);
        load.evaluateResidual();

        success *= load.isClosed();
        success *= isEqual(current[0], static_cast<ScalarT>(2.0), tol);
        success *= isEqual(current[1], static_cast<ScalarT>(-4.0), tol);
        success *= isEqual(current[2], static_cast<ScalarT>(6.0), tol);

        return success.report(__func__);
      }

    private:
      static auto makeBusData() -> BusDataT
      {
        BusDataT data;
        data.v0 = {1.0, -2.0, 3.0};
        return data;
      }

      static auto makeLoadData(bool closed) -> LoadDataT
      {
        LoadDataT data;
        data.conductance = {2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0};
        data.closed      = closed;
        return data;
      }

      static void prepare(EMT::Bus<ScalarT, IdxT>&       bus,
                          EMT::ShuntLoad<ScalarT, IdxT>& load)
      {
        bus.allocate();
        load.allocate();

        bus.setIndexRanges({0, 3}, {0, 3});
        load.setIndexRanges({3, 0}, {3, 0});

        typename EMT::Component<ScalarT, IdxT>::TerminalView terminal;
        terminal.y              = bus.y().data();
        terminal.yp             = bus.yp().data();
        terminal.variable_index = bus.variableIndex(0);
        terminal.residual_index = bus.residualIndex(0);
        load.bindTerminal(0, terminal);
      }
    };

  } // namespace Testing
} // namespace GridKit

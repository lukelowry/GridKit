#pragma once

#include <limits>

#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Testing/TestHelpers.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class BusTests
    {
    public:
      using RealT = typename GridKit::ScalarTraits<ScalarT>::RealT;
      using DataT = EMT::BusData<RealT, IdxT>;

      TestOutcome constructor()
      {
        TestStatus success = true;

        EMT::Bus<ScalarT, IdxT> bus(makeData());

        success *= (bus.verify() == 0);
        success *= (bus.stateCount() == 3);
        success *= (bus.allocate() == 0);
        success *= (bus.size() == 3);

        return success.report(__func__);
      }

      TestOutcome initialization()
      {
        TestStatus success = true;

        const auto              data = makeData();
        EMT::Bus<ScalarT, IdxT> bus(data);
        bus.allocate();
        bus.initialize();

        const auto tol = static_cast<RealT>(1.0e-12);
        for (size_t phase = 0; phase < EMT::Bus<ScalarT, IdxT>::PHASE_COUNT; ++phase)
        {
          success *= isEqual(bus.y()[phase], static_cast<ScalarT>(data.v0[phase]), tol);
          success *= isEqual(bus.yp()[phase], static_cast<ScalarT>(data.vp0[phase]), tol);
        }

        return success.report(__func__);
      }

      TestOutcome residual()
      {
        TestStatus success = true;

        EMT::Bus<ScalarT, IdxT> bus(makeData());
        bus.allocate();
        bus.initialize();
        bus.evaluateResidual();

        const auto tol = static_cast<RealT>(1.0e-12);
        for (auto value : bus.getResidual())
        {
          success *= isEqual(value, static_cast<ScalarT>(0.0), tol);
        }

        return success.report(__func__);
      }

      TestOutcome tagDifferentiable()
      {
        TestStatus success = true;

        EMT::Bus<ScalarT, IdxT> bus(makeData());
        bus.allocate();
        bus.tagDifferentiable();

        success *= !bus.tag()[0];
        success *= !bus.tag()[1];
        success *= !bus.tag()[2];

        return success.report(__func__);
      }

      TestOutcome jacobian()
      {
        TestStatus success = true;

        EMT::Bus<ScalarT, IdxT> bus(makeData());
        bus.allocate();
        bus.evaluateJacobian();

        success *= (bus.nnz() == 0);
        success *= (bus.getJacobian().nnz() == 0);

        return success.report(__func__);
      }

    private:
      static auto makeData() -> DataT
      {
        DataT data;
        data.v0  = {1.0, -0.2, 0.4};
        data.vp0 = {0.1, 0.05, -0.02};
        return data;
      }
    };

  } // namespace Testing
} // namespace GridKit

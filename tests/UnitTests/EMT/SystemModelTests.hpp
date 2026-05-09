#pragma once

#include <limits>
#include <stdexcept>

#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Testing/TestHelpers.hpp>
#include <GridKit/Testing/Testing.hpp>
#include <GridKit/Utilities/MapFromCOO.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class SystemModelTests
    {
    public:
      using RealT     = typename GridKit::ScalarTraits<ScalarT>::RealT;
      using BusDataT  = EMT::BusData<RealT, IdxT>;
      using LineDataT = EMT::LineData<RealT, IdxT>;

      TestOutcome ownershipAndAllocation()
      {
        TestStatus success = true;

        EMT::SystemModel<ScalarT, IdxT> system;
        auto&                           bus1  = system.addBus(makeBus1Data());
        auto&                           bus2  = system.addBus(makeBus2Data());
        auto&                           line  = system.addLine(makeDynamicLineData());
        success                              *= (system.connect(line.terminal(0), bus1) == 0);
        success                              *= (system.connect(line.terminal(1), bus2) == 0);

        success *= (system.allocate() == 0);
        success *= (system.size() == 12);

        success *= (bus1.variableRange().begin == 0);
        success *= (bus1.variableRange().size == 3);
        success *= (bus2.variableRange().begin == 3);
        success *= (bus2.variableRange().size == 3);
        success *= (line.variableRange().begin == 6);
        success *= (line.variableRange().size == 6);

        bool add_bus_threw = false;
        try
        {
          system.addBus(makeBus1Data());
        }
        catch (const std::logic_error&)
        {
          add_bus_threw = true;
        }
        success *= add_bus_threw;

        bool add_line_threw = false;
        try
        {
          system.addLine(makeDirectLineData());
        }
        catch (const std::logic_error&)
        {
          add_line_threw = true;
        }
        success *= add_line_threw;

        bool connect_threw = false;
        try
        {
          system.connect(line.terminal(0), bus1);
        }
        catch (const std::logic_error&)
        {
          connect_threw = true;
        }
        success *= connect_threw;

        EMT::SystemModel<ScalarT, IdxT> incomplete_system;
        auto&                           incomplete_bus  = incomplete_system.addBus(makeBus1Data());
        auto&                           incomplete_line = incomplete_system.addLine(makeDynamicLineData());
        incomplete_system.connect(incomplete_line.terminal(0), incomplete_bus);
        success *= (incomplete_system.allocate() > 0);

        EMT::SystemModel<ScalarT, IdxT> external_terminal_system;
        auto&                           external_terminal_bus = external_terminal_system.addBus(makeBus1Data());
        EMT::Line<ScalarT, IdxT>        external_line(makeDirectLineData());
        external_terminal_system.connect(external_line.terminal(0), external_terminal_bus);
        success *= (external_terminal_system.allocate() > 0);

        EMT::SystemModel<ScalarT, IdxT> external_bus_system;
        EMT::Bus<ScalarT, IdxT>         external_bus(makeBus1Data());
        auto&                           external_bus_line = external_bus_system.addLine(makeDirectLineData());
        external_bus_system.connect(external_bus_line.terminal(0), external_bus);
        external_bus_system.connect(external_bus_line.terminal(1), external_bus);
        success *= (external_bus_system.allocate() > 0);

        return success.report(__func__);
      }

      TestOutcome initialization()
      {
        TestStatus success = true;

        EMT::SystemModel<ScalarT, IdxT> system;
        auto&                           bus1 = system.addBus(makeBus1Data());
        auto&                           bus2 = system.addBus(makeBus2Data());
        auto&                           line = system.addLine(makeDynamicLineData());
        system.connect(line.terminal(0), bus1);
        system.connect(line.terminal(1), bus2);

        system.allocate();
        system.initialize();
        line.evaluateResidual();

        const auto bus1_data = makeBus1Data();
        const auto bus2_data = makeBus2Data();
        const auto tol       = static_cast<RealT>(1.0e-12);

        for (size_t phase = 0; phase < 3; ++phase)
        {
          success *= isEqual(system.y()[phase], static_cast<ScalarT>(bus1_data.v0[phase]), tol);
          success *= isEqual(system.y()[3 + phase], static_cast<ScalarT>(bus2_data.v0[phase]), tol);
          success *= isEqual(system.yp()[phase], static_cast<ScalarT>(bus1_data.vp0[phase]), tol);
          success *= isEqual(system.yp()[3 + phase], static_cast<ScalarT>(bus2_data.vp0[phase]), tol);
        }

        for (auto value : line.getResidual())
        {
          success *= isEqual(value, static_cast<ScalarT>(0.0), tol);
        }

        return success.report(__func__);
      }

      TestOutcome residualDispatch()
      {
        TestStatus success = true;

        EMT::SystemModel<ScalarT, IdxT> system;
        auto&                           bus1 = system.addBus(makeBus1Data());
        auto&                           bus2 = system.addBus(makeBus2Data());
        auto&                           line = system.addLine(makeDirectLineData());
        system.connect(line.terminal(0), bus1);
        system.connect(line.terminal(1), bus2);

        system.allocate();
        system.initialize();

        system.y()[0] = 0.5;
        system.y()[1] = -0.25;
        system.y()[2] = 0.75;
        system.y()[3] = -0.1;
        system.y()[4] = 0.4;
        system.y()[5] = -0.2;

        system.evaluateResidual();

        const std::vector<ScalarT> expected{-0.4875, 0.6125, -0.6975, 0.4875, -0.6125, 0.6975};
        const auto                 tol = static_cast<RealT>(1.0e-12);
        for (size_t i = 0; i < expected.size(); ++i)
        {
          success *= isEqual(system.getResidual()[i], expected[i], tol);
        }

        return success.report(__func__);
      }

      TestOutcome tagDifferentiable()
      {
        TestStatus success = true;

        EMT::SystemModel<ScalarT, IdxT> algebraic_system;
        auto&                           bus1 = algebraic_system.addBus(makeBus1Data());
        auto&                           bus2 = algebraic_system.addBus(makeBus2Data());
        auto&                           line = algebraic_system.addLine(makeDynamicLineData());
        algebraic_system.connect(line.terminal(0), bus1);
        algebraic_system.connect(line.terminal(1), bus2);
        algebraic_system.allocate();
        algebraic_system.tagDifferentiable();

        for (size_t phase = 0; phase < 6; ++phase)
        {
          success *= !algebraic_system.tag()[phase];
        }
        for (size_t state = 6; state < 12; ++state)
        {
          success *= algebraic_system.tag()[state];
        }

        EMT::SystemModel<ScalarT, IdxT> derivative_system;
        auto&                           derivative_bus1 = derivative_system.addBus(makeBus1Data());
        auto&                           derivative_bus2 = derivative_system.addBus(makeBus2Data());
        auto                            derivative_line = makeDynamicLineData();
        derivative_line.characteristic_admittance.e[0]  = 0.01;
        auto& derivative_line_component                 = derivative_system.addLine(derivative_line);
        derivative_system.connect(derivative_line_component.terminal(0), derivative_bus1);
        derivative_system.connect(derivative_line_component.terminal(1), derivative_bus2);
        derivative_system.allocate();
        derivative_system.tagDifferentiable();

        for (auto value : derivative_system.tag())
        {
          success *= value;
        }

        return success.report(__func__);
      }

      TestOutcome jacobianAssembly()
      {
        TestStatus success = true;

        EMT::SystemModel<ScalarT, IdxT> system;
        auto&                           bus1 = system.addBus(makeBus1Data());
        auto&                           bus2 = system.addBus(makeBus2Data());
        auto&                           line = system.addLine(makeDirectLineData());
        system.connect(line.terminal(0), bus1);
        system.connect(line.terminal(1), bus2);

        system.allocate();
        system.updateTime(0.0, 10.0);
        system.evaluateJacobian();

        auto jacobian = MapFromCOO<RealT, IdxT>(system.getJacobian());

        std::vector<DependencyTracking::Variable::DependencyMap> expected(6);
        fillExpectedDirectBlock(expected, 0, 0, 3);
        fillExpectedDirectBlock(expected, 3, 3, 0);

        const auto tol = static_cast<RealT>(100.0) * std::numeric_limits<RealT>::epsilon();
        for (size_t row = 0; row < expected.size(); ++row)
        {
          success *= isEqual<IdxT, RealT>(jacobian[row], expected[row], tol);
        }
        success *= (system.getCsrJacobian() != nullptr);
        success *= (system.nnz() == 36);

        system.evaluateJacobian();
        success *= (system.getCsrJacobian() != nullptr);
        success *= (system.nnz() == 36);

        return success.report(__func__);
      }

    private:
      static auto makeBus1Data() -> BusDataT
      {
        BusDataT data;
        data.v0  = {1.0, -0.2, 0.4};
        data.vp0 = {0.1, 0.05, -0.02};
        return data;
      }

      static auto makeBus2Data() -> BusDataT
      {
        BusDataT data;
        data.v0  = {-0.1, 0.3, 0.8};
        data.vp0 = {-0.04, 0.02, 0.06};
        return data;
      }

      static auto makeDirectLineData() -> LineDataT
      {
        LineDataT data;
        auto&     yc = data.characteristic_admittance;
        yc.dimension = 3;
        yc.d         = {1.0, 0.1, -0.05, 0.2, 1.2, 0.05, -0.1, 0.15, 0.9};
        yc.e.assign(9, 0.0);
        return data;
      }

      static auto makeDynamicLineData() -> LineDataT
      {
        auto  data = makeDirectLineData();
        auto& yc   = data.characteristic_admittance;

        yc.p = {-2.0};
        yc.b = {0.5, -0.25, 0.75};
        yc.c = {1.0, -0.5, 0.25};

        yc.complex_p_real = {-1.0};
        yc.complex_p_imag = {4.0};
        yc.complex_b_real = {0.2, 0.1, -0.3};
        yc.complex_b_imag = {-0.4, 0.25, 0.5};
        yc.complex_c_real = {0.6, -0.2, 0.4};
        yc.complex_c_imag = {0.1, 0.3, -0.5};

        return data;
      }

      static void fillExpectedDirectBlock(std::vector<DependencyTracking::Variable::DependencyMap>& expected,
                                          size_t                                                    row_offset,
                                          IdxT                                                      col_offset,
                                          IdxT                                                      remote_col_offset)
      {
        expected[row_offset + 0][col_offset + 0]        = -1.0;
        expected[row_offset + 0][col_offset + 1]        = -0.1;
        expected[row_offset + 0][col_offset + 2]        = 0.05;
        expected[row_offset + 0][remote_col_offset + 0] = 1.0;
        expected[row_offset + 0][remote_col_offset + 1] = 0.1;
        expected[row_offset + 0][remote_col_offset + 2] = -0.05;

        expected[row_offset + 1][col_offset + 0]        = -0.2;
        expected[row_offset + 1][col_offset + 1]        = -1.2;
        expected[row_offset + 1][col_offset + 2]        = -0.05;
        expected[row_offset + 1][remote_col_offset + 0] = 0.2;
        expected[row_offset + 1][remote_col_offset + 1] = 1.2;
        expected[row_offset + 1][remote_col_offset + 2] = 0.05;

        expected[row_offset + 2][col_offset + 0]        = 0.1;
        expected[row_offset + 2][col_offset + 1]        = -0.15;
        expected[row_offset + 2][col_offset + 2]        = -0.9;
        expected[row_offset + 2][remote_col_offset + 0] = -0.1;
        expected[row_offset + 2][remote_col_offset + 1] = 0.15;
        expected[row_offset + 2][remote_col_offset + 2] = 0.9;
      }
    };

  } // namespace Testing
} // namespace GridKit

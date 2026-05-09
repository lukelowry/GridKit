#pragma once

#include <array>
#include <limits>

#include <GridKit/Model/EMT/Bus/Bus.hpp>
#include <GridKit/Model/EMT/Line/Line.hpp>
#include <GridKit/Testing/TestHelpers.hpp>
#include <GridKit/Testing/Testing.hpp>
#include <GridKit/Utilities/MapFromCOO.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class LineTests
    {
    public:
      using RealT     = typename GridKit::ScalarTraits<ScalarT>::RealT;
      using BusDataT  = EMT::BusData<RealT, IdxT>;
      using LineDataT = EMT::LineData<RealT, IdxT>;

      TestOutcome constructor()
      {
        TestStatus success = true;

        EMT::Line<ScalarT, IdxT> line(makeDynamicLineData());

        success *= (line.verify() == 0);
        success *= (line.terminalCount() == 2);
        success *= (line.terminalStateCount() == 3);
        success *= (line.stateCount() == 6);
        success *= (line.allocate() == 0);
        success *= (line.size() == 6);

        EMT::Line<ScalarT, IdxT> missing_terminals;
        missing_terminals.allocate();
        success *= (missing_terminals.initialize() > 0);

        auto bad_data                                = makeDirectLineData();
        bad_data.characteristic_admittance.dimension = 2;
        bad_data.characteristic_admittance.d.assign(4, 0.0);
        bad_data.characteristic_admittance.e.assign(4, 0.0);
        EMT::Line<ScalarT, IdxT> bad_line(bad_data);
        success *= (bad_line.verify() > 0);

        return success.report(__func__);
      }

      TestOutcome initialization()
      {
        TestStatus success = true;

        EMT::Bus<ScalarT, IdxT>  bus1(makeBus1Data());
        EMT::Bus<ScalarT, IdxT>  bus2(makeBus2Data());
        EMT::Line<ScalarT, IdxT> line(makeDynamicLineData());

        bus1.allocate();
        bus2.allocate();
        line.allocate();

        bindLine(line, bus1, bus2);

        bus1.initialize();
        bus2.initialize();
        line.initialize();
        line.evaluateResidual();

        const auto tol = static_cast<RealT>(1.0e-12);
        for (auto value : line.getResidual())
        {
          success *= isEqual(value, static_cast<ScalarT>(0.0), tol);
        }

        return success.report(__func__);
      }

      TestOutcome residual()
      {
        TestStatus success = true;

        EMT::Bus<ScalarT, IdxT>  bus1(makeBus1Data());
        EMT::Bus<ScalarT, IdxT>  bus2(makeBus2Data());
        EMT::Line<ScalarT, IdxT> line(makeDirectLineData());

        bus1.allocate();
        bus2.allocate();
        line.allocate();

        bus1.setIndexRanges({0, 3}, {0, 3});
        bus2.setIndexRanges({3, 3}, {3, 3});
        line.setIndexRanges({6, 0}, {6, 0});
        bindLine(line, bus1, bus2);

        bus1.y() = {0.5, -0.25, 0.75};
        bus2.y() = {-0.1, 0.4, -0.2};
        bus1.yp().assign(3, 0.0);
        bus2.yp().assign(3, 0.0);

        line.evaluateResidual();

        std::vector<ScalarT> residual(6, 0.0);
        assembleLineCurrent(line, bus1, bus2, residual);

        const std::vector<ScalarT> expected{-0.4875, 0.6125, -0.6975, 0.4875, -0.6125, 0.6975};
        const auto                 tol = static_cast<RealT>(1.0e-12);
        for (size_t i = 0; i < expected.size(); ++i)
        {
          success *= isEqual(residual[i], expected[i], tol);
        }

        return success.report(__func__);
      }

      TestOutcome tagDifferentiable()
      {
        TestStatus success = true;

        EMT::Bus<ScalarT, IdxT>  bus1(makeBus1Data());
        EMT::Bus<ScalarT, IdxT>  bus2(makeBus2Data());
        EMT::Line<ScalarT, IdxT> line(makeDynamicLineData());

        bus1.allocate();
        bus2.allocate();
        line.allocate();
        bus1.setIndexRanges({0, 3}, {0, 3});
        bus2.setIndexRanges({3, 3}, {3, 3});
        line.setIndexRanges({6, 6}, {6, 6});
        bindLine(line, bus1, bus2);

        std::vector<bool> tag(12, false);
        bus1.tagDifferentiable();
        bus2.tagDifferentiable();
        line.tagDifferentiable();
        bus1.stampDifferentiable(tag);
        bus2.stampDifferentiable(tag);
        line.stampDifferentiable(tag);

        for (size_t phase = 0; phase < 6; ++phase)
        {
          success *= !tag[phase];
        }
        for (size_t state = 6; state < 12; ++state)
        {
          success *= tag[state];
        }

        auto derivative_data                           = makeDynamicLineData();
        derivative_data.characteristic_admittance.e[0] = 0.01;
        EMT::Line<ScalarT, IdxT> derivative_line(derivative_data);
        derivative_line.allocate();
        derivative_line.setIndexRanges({6, 6}, {6, 6});
        bindLine(derivative_line, bus1, bus2);

        std::vector<bool> derivative_tag(12, false);
        derivative_line.tagDifferentiable();
        derivative_line.stampDifferentiable(derivative_tag);
        for (size_t state = 6; state < 12; ++state)
        {
          success *= derivative_tag[state];
        }
        success *= derivative_line.terminalHasDerivativeFeedthrough(0);
        success *= derivative_line.terminalHasDerivativeFeedthrough(1);

        return success.report(__func__);
      }

      TestOutcome jacobian()
      {
        TestStatus success = true;

        EMT::Bus<ScalarT, IdxT>  bus1(makeBus1Data());
        EMT::Bus<ScalarT, IdxT>  bus2(makeBus2Data());
        EMT::Line<ScalarT, IdxT> line(makeDynamicLineData());

        bus1.allocate();
        bus2.allocate();
        line.allocate();
        bus1.setIndexRanges({0, 3}, {0, 3});
        bus2.setIndexRanges({3, 3}, {3, 3});
        line.setIndexRanges({6, 6}, {6, 6});
        bindLine(line, bus1, bus2);

        line.updateTime(0.0, 10.0);
        line.evaluateJacobian();

        auto jacobian = MapFromCOO<RealT, IdxT>(line.getJacobian());

        std::vector<DependencyTracking::Variable::DependencyMap> expected(12);
        fillExpectedTerminalBlock(expected, 0, 0, 6, 3, 9);
        fillExpectedTerminalBlock(expected, 3, 3, 9, 0, 6);

        const auto tol = static_cast<RealT>(100.0) * std::numeric_limits<RealT>::epsilon();
        for (size_t row = 0; row < expected.size(); ++row)
        {
          success *= isEqual<IdxT, RealT>(jacobian[row], expected[row], tol);
        }

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

      static void bindLine(EMT::Line<ScalarT, IdxT>& line,
                           EMT::Bus<ScalarT, IdxT>&  bus1,
                           EMT::Bus<ScalarT, IdxT>&  bus2)
      {
        typename EMT::Line<ScalarT, IdxT>::TerminalView terminal1;
        terminal1.y              = bus1.y().data();
        terminal1.yp             = bus1.yp().data();
        terminal1.variable_index = bus1.variableIndex(0);
        terminal1.residual_index = bus1.residualIndex(0);

        typename EMT::Line<ScalarT, IdxT>::TerminalView terminal2;
        terminal2.y              = bus2.y().data();
        terminal2.yp             = bus2.yp().data();
        terminal2.variable_index = bus2.variableIndex(0);
        terminal2.residual_index = bus2.residualIndex(0);

        line.bindTerminal(0, terminal1);
        line.bindTerminal(1, terminal2);
      }

      static void assembleLineCurrent(const EMT::Line<ScalarT, IdxT>& line,
                                      const EMT::Bus<ScalarT, IdxT>&  bus1,
                                      const EMT::Bus<ScalarT, IdxT>&  bus2,
                                      std::vector<ScalarT>&           residual)
      {
        const auto* current1 = line.terminalCurrent(0);
        const auto* current2 = line.terminalCurrent(1);
        for (size_t phase = 0; phase < EMT::Bus<ScalarT, IdxT>::PHASE_COUNT; ++phase)
        {
          residual[static_cast<size_t>(bus1.residualIndex(phase))] -= current1[phase];
          residual[static_cast<size_t>(bus2.residualIndex(phase))] -= current2[phase];
        }
      }

      static void fillExpectedTerminalBlock(std::vector<DependencyTracking::Variable::DependencyMap>& expected,
                                            size_t                                                    row_offset,
                                            IdxT                                                      input_col_offset,
                                            IdxT                                                      state_col_offset,
                                            IdxT                                                      remote_input_col_offset,
                                            IdxT                                                      remote_state_col_offset)
      {
        expected[row_offset + 0][input_col_offset + 0]        = -1.0;
        expected[row_offset + 0][input_col_offset + 1]        = -0.1;
        expected[row_offset + 0][input_col_offset + 2]        = 0.05;
        expected[row_offset + 0][state_col_offset + 0]        = -1.0;
        expected[row_offset + 0][state_col_offset + 1]        = -1.2;
        expected[row_offset + 0][state_col_offset + 2]        = 0.2;
        expected[row_offset + 0][remote_input_col_offset + 0] = 1.0;
        expected[row_offset + 0][remote_input_col_offset + 1] = 0.1;
        expected[row_offset + 0][remote_input_col_offset + 2] = -0.05;
        expected[row_offset + 0][remote_state_col_offset + 0] = 1.0;
        expected[row_offset + 0][remote_state_col_offset + 1] = 1.2;
        expected[row_offset + 0][remote_state_col_offset + 2] = -0.2;

        expected[row_offset + 1][input_col_offset + 0]        = -0.2;
        expected[row_offset + 1][input_col_offset + 1]        = -1.2;
        expected[row_offset + 1][input_col_offset + 2]        = -0.05;
        expected[row_offset + 1][state_col_offset + 0]        = 0.5;
        expected[row_offset + 1][state_col_offset + 1]        = 0.4;
        expected[row_offset + 1][state_col_offset + 2]        = 0.6;
        expected[row_offset + 1][remote_input_col_offset + 0] = 0.2;
        expected[row_offset + 1][remote_input_col_offset + 1] = 1.2;
        expected[row_offset + 1][remote_input_col_offset + 2] = 0.05;
        expected[row_offset + 1][remote_state_col_offset + 0] = -0.5;
        expected[row_offset + 1][remote_state_col_offset + 1] = -0.4;
        expected[row_offset + 1][remote_state_col_offset + 2] = -0.6;

        expected[row_offset + 2][input_col_offset + 0]        = 0.1;
        expected[row_offset + 2][input_col_offset + 1]        = -0.15;
        expected[row_offset + 2][input_col_offset + 2]        = -0.9;
        expected[row_offset + 2][state_col_offset + 0]        = -0.25;
        expected[row_offset + 2][state_col_offset + 1]        = -0.8;
        expected[row_offset + 2][state_col_offset + 2]        = -1.0;
        expected[row_offset + 2][remote_input_col_offset + 0] = -0.1;
        expected[row_offset + 2][remote_input_col_offset + 1] = 0.15;
        expected[row_offset + 2][remote_input_col_offset + 2] = 0.9;
        expected[row_offset + 2][remote_state_col_offset + 0] = 0.25;
        expected[row_offset + 2][remote_state_col_offset + 1] = 0.8;
        expected[row_offset + 2][remote_state_col_offset + 2] = 1.0;

        expected[row_offset + 6][input_col_offset + 0] = 0.5;
        expected[row_offset + 6][input_col_offset + 1] = -0.25;
        expected[row_offset + 6][input_col_offset + 2] = 0.75;
        expected[row_offset + 6][state_col_offset + 0] = -12.0;

        expected[row_offset + 7][input_col_offset + 0] = 0.2;
        expected[row_offset + 7][input_col_offset + 1] = 0.1;
        expected[row_offset + 7][input_col_offset + 2] = -0.3;
        expected[row_offset + 7][state_col_offset + 1] = -11.0;
        expected[row_offset + 7][state_col_offset + 2] = -4.0;

        expected[row_offset + 8][input_col_offset + 0] = -0.4;
        expected[row_offset + 8][input_col_offset + 1] = 0.25;
        expected[row_offset + 8][input_col_offset + 2] = 0.5;
        expected[row_offset + 8][state_col_offset + 1] = 4.0;
        expected[row_offset + 8][state_col_offset + 2] = -11.0;
      }
    };

  } // namespace Testing
} // namespace GridKit

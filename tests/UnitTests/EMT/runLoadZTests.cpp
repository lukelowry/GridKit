#include <array>
#include <vector>

#include <GridKit/Model/EMT/Component/Load/LoadZ/LoadZImpl.hpp>
#include <GridKit/Testing/Testing.hpp>

#include "PhysicalModelTestUtils.hpp"

namespace
{
  using namespace GridKit;
  using namespace GridKit::Testing;
  using namespace GridKit::Testing::EMTTest;
  using LoadT = EMT::LoadZ<double, std::size_t>;

  double csrValue(SystemT::CsrMatrixT& jacobian,
                  std::size_t          row,
                  std::size_t          column)
  {
    const auto* row_data = jacobian.getRowData();
    const auto* columns  = jacobian.getColData();
    const auto* values   = jacobian.getValues();
    for (std::size_t entry = row_data[row]; entry < row_data[row + 1]; ++entry)
    {
      if (columns[entry] == column)
      {
        return values[entry];
      }
    }
    return 0.0;
  }

  bool captureGateJacobian(SystemT&                  system,
                           LoadT&                    load,
                           double                    enable_value,
                           std::vector<std::size_t>& row_data,
                           std::vector<std::size_t>& columns,
                           std::array<double, 3>&    gate_values)
  {
    system.getSignal(1)->init(enable_value);
    system.updateTime(0.0, 23.0);
    if (system.evaluateResidual() != 0 || system.evaluateJacobian() != 0)
    {
      return false;
    }

    auto* jacobian = system.getCsrJacobian();
    if (jacobian == nullptr)
    {
      return false;
    }

    row_data.assign(jacobian->getRowData(),
                    jacobian->getRowData() + jacobian->getNumRows() + 1);
    columns.assign(jacobian->getColData(),
                   jacobian->getColData() + jacobian->getNnz());

    auto* bus      = system.getBus(632);
    gate_values[0] = csrValue(*jacobian,
                              bus->getResidualIndex(0),
                              load.getVariableIndex(0));
    gate_values[1] = csrValue(*jacobian,
                              bus->getResidualIndex(1),
                              load.getVariableIndex(1));
    gate_values[2] = csrValue(*jacobian,
                              bus->getResidualIndex(2),
                              load.getVariableIndex(2));
    return true;
  }

  EMT::ABCMatrix<double> coupledResistance()
  {
    return {{{2.0, 0.20, 0.10},
             {0.20, 2.50, 0.15},
             {0.10, 0.15, 3.00}}};
  }

  EMT::ABCMatrix<double> coupledInductance()
  {
    return {{{0.020, 0.0020, 0.0010},
             {0.0020, 0.025, 0.0015},
             {0.0010, 0.0015, 0.030}}};
  }

  TestOutcome zeroInitializationAndInjectionGate()
  {
    TestStatus success  = true;
    const auto data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);
    auto system         = makeFixtureSystem(data);

    auto* normal_load  = findComponent<LoadT>(*system, data, 0);
    auto* fault_load   = findComponent<LoadT>(*system, data, 1);
    success           *= normal_load != nullptr && fault_load != nullptr;
    if (normal_load == nullptr || fault_load == nullptr)
    {
      return success.report(__func__);
    }

    success *= normal_load->size() == 3 && fault_load->size() == 3;
    for (std::size_t index = 0; index < normal_load->size(); ++index)
    {
      normal_load->y().getData()[index]  = 1.0;
      normal_load->yp().getData()[index] = -1.0;
      fault_load->y().getData()[index]   = 2.0;
      fault_load->yp().getData()[index]  = -2.0;
    }
    success *= normal_load->initialize() == 0;
    success *= fault_load->initialize() == 0;
    for (std::size_t index = 0; index < normal_load->size(); ++index)
    {
      success *= normal_load->y().getData()[index] == 0.0;
      success *= normal_load->yp().getData()[index] == 0.0;
      success *= fault_load->y().getData()[index] == 0.0;
      success *= fault_load->yp().getData()[index] == 0.0;
    }
    success *= normal_load->tag()[0] && normal_load->tag()[1]
               && normal_load->tag()[2];

    auto* bus                    = system->getBus(632);
    auto* enable                 = system->getSignal(1);
    fault_load->y().getData()[0] = 0.70;
    fault_load->y().getData()[1] = -1.10;
    fault_load->y().getData()[2] = 1.40;
    fault_load->y().setDataUpdated();
    const double ia = fault_load->y().getData()[0];
    const double ib = fault_load->y().getData()[1];
    const double ic = fault_load->y().getData()[2];

    enable->init(0.0);
    bus->evaluateResidual();
    fault_load->evaluateResidual();
    success *= bus->Ia() == 0.0 && bus->Ib() == 0.0 && bus->Ic() == 0.0;

    enable->init(1.0);
    bus->evaluateResidual();
    fault_load->evaluateResidual();
    success *= isEqual(bus->Ia(), ia, 1.0e-13);
    success *= isEqual(bus->Ib(), ib, 1.0e-13);
    success *= isEqual(bus->Ic(), ic, 1.0e-13);

    enable->init(0.0);
    bus->evaluateResidual();
    fault_load->evaluateResidual();
    success *= bus->Ia() == 0.0 && bus->Ib() == 0.0 && bus->Ic() == 0.0;
    success *= isEqual(fault_load->y().getData()[0], ia, 1.0e-15);
    success *= isEqual(fault_load->y().getData()[1], ib, 1.0e-15);
    success *= isEqual(fault_load->y().getData()[2], ic, 1.0e-15);

    auto       coupled_data                          = data.loadz[0];
    const auto R                                     = coupledResistance();
    const auto L                                     = coupledInductance();
    coupled_data.parameters[EMT::LoadZParameters::R] = R;
    coupled_data.parameters[EMT::LoadZParameters::L] = L;
    LoadT coupled_load(system->getBus(670), coupled_data);
    coupled_load.getSignals()
        .template attachSignalNode<EMT::LoadZExternalVariables::enable>(
            system->getSignal(0));
    success *= coupled_load.allocate() == 0;
    success *= coupled_load.verify() == 0;

    const std::array<double, 3> y{0.70, -1.10, 1.40};
    const std::array<double, 3> yp{-2.00, 2.30, -2.60};
    const std::array<double, 3> wb{120.0, -230.0, 340.0};
    std::array<double, 3>       f{};
    coupled_load.evaluateInternalResidual(y.data(), yp.data(), wb.data(), f.data());
    for (std::size_t row = 0; row < 3; ++row)
    {
      double expected = wb[row];
      for (std::size_t column = 0; column < 3; ++column)
      {
        expected += R[row][column] * y[column]
                    + L[row][column] * yp[column];
      }
      success *= isEqual(f[row], expected, 2.0e-13);
    }

    std::array<double, 3> h{};
    coupled_load.evaluateBusResidual(y.data(), 0.35, h.data());
    success *= isEqual(h[0], 0.35 * y[0], 1.0e-15);
    success *= isEqual(h[1], 0.35 * y[1], 1.0e-15);
    success *= isEqual(h[2], 0.35 * y[2], 1.0e-15);

    return success.report(__func__);
  }

  TestOutcome gateKeepsFixedJacobianPattern()
  {
    TestStatus success                                 = true;
    auto       data                                    = loadFixtureData();
    success                                           *= isThreeBusMutuallyCoupled(data);
    const auto R                                       = coupledResistance();
    const auto L                                       = coupledInductance();
    data.loadz[0].parameters[EMT::LoadZParameters::R]  = R;
    data.loadz[0].parameters[EMT::LoadZParameters::L]  = L;
    data.loadz[1].parameters[EMT::LoadZParameters::R]  = R;
    data.loadz[1].parameters[EMT::LoadZParameters::L]  = L;
    auto  system                                       = makeFixtureSystem(data);
    auto* normal_load                                  = findComponent<LoadT>(*system, data, 0);
    auto* fault_load                                   = findComponent<LoadT>(*system, data, 1);
    success                                           *= normal_load != nullptr;
    success                                           *= fault_load != nullptr;
    if (normal_load == nullptr || fault_load == nullptr)
    {
      return success.report(__func__);
    }

#ifdef GRIDKIT_ENABLE_ENZYME
    std::vector<std::size_t> off_rows;
    std::vector<std::size_t> off_columns;
    std::array<double, 3>    off_values{};
    success *= captureGateJacobian(*system,
                                   *fault_load,
                                   0.0,
                                   off_rows,
                                   off_columns,
                                   off_values);

    std::vector<std::size_t> on_rows;
    std::vector<std::size_t> on_columns;
    std::array<double, 3>    on_values{};
    success *= captureGateJacobian(*system,
                                   *fault_load,
                                   1.0,
                                   on_rows,
                                   on_columns,
                                   on_values);

    std::vector<std::size_t> off_again_rows;
    std::vector<std::size_t> off_again_columns;
    std::array<double, 3>    off_again_values{};
    success *= captureGateJacobian(*system,
                                   *fault_load,
                                   0.0,
                                   off_again_rows,
                                   off_again_columns,
                                   off_again_values);

    success *= off_rows == on_rows && on_rows == off_again_rows;
    success *= off_columns == on_columns && on_columns == off_again_columns;
    success *= off_values[0] == 0.0 && off_values[1] == 0.0
               && off_values[2] == 0.0;
    success *= on_values[0] == 1.0 && on_values[1] == 1.0
               && on_values[2] == 1.0;
    success *= off_again_values[0] == 0.0 && off_again_values[1] == 0.0
               && off_again_values[2] == 0.0;

    success *= componentJacobianMatchesReferences(
        *system, *normal_load, data, {7.0, 23.0});
    success *= componentJacobianMatchesReferences(
        *system, *fault_load, data, {7.0, 23.0});
#endif

    return success.report(__func__);
  }
} // namespace

int main()
{
  GridKit::Testing::TestingResults result;
  result += zeroInitializationAndInjectionGate();
  result += gateKeepsFixedJacobianPattern();
  return result.summary();
}

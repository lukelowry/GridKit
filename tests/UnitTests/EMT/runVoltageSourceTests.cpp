#include <array>
#include <cmath>

#include <GridKit/Model/EMT/Component/Source/VoltageSource/VoltageSourceImpl.hpp>
#include <GridKit/Testing/Testing.hpp>

#include "PhysicalModelTestUtils.hpp"

namespace
{
  using namespace GridKit;
  using namespace GridKit::Testing;
  using namespace GridKit::Testing::EMTTest;
  using SourceT = EMT::VoltageSource<double, std::size_t>;

  EMT::ABCMatrix<double> coupledResistance()
  {
    return {{{0.40, 0.04, 0.02},
             {0.04, 0.50, 0.03},
             {0.02, 0.03, 0.60}}};
  }

  EMT::ABCMatrix<double> coupledInductance()
  {
    return {{{0.0040, 0.0004, 0.0002},
             {0.0004, 0.0050, 0.0003},
             {0.0002, 0.0003, 0.0060}}};
  }

  TestOutcome zeroInitializationAndExplicitSourceRows()
  {
    TestStatus success  = true;
    const auto data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);
    auto  system        = makeFixtureSystem(data);
    auto* source        = findComponent<SourceT>(*system, data);
    success            *= source != nullptr;
    if (source == nullptr)
    {
      return success.report(__func__);
    }

    success *= source->size() == 6;
    for (std::size_t index = 0; index < source->size(); ++index)
    {
      source->y().getData()[index]  = 1.0;
      source->yp().getData()[index] = -1.0;
    }
    success *= source->initialize() == 0;
    for (std::size_t index = 0; index < source->size(); ++index)
    {
      success *= source->y().getData()[index] == 0.0;
      success *= source->yp().getData()[index] == 0.0;
    }
    success *= source->tag()[0] && source->tag()[1] && source->tag()[2];
    success *= !source->tag()[3] && !source->tag()[4] && !source->tag()[5];

    const auto& source_data = data.voltage_source[0];
    const auto& E           = std::get<EMT::ABCVector<double>>(
        source_data.parameters.at(EMT::VoltageSourceParameters::E));
    const auto& phi = std::get<EMT::ABCVector<double>>(
        source_data.parameters.at(EMT::VoltageSourceParameters::phi));
    const double sqrt_two = std::sqrt(2.0);

    std::array<double, 6> y{};
    std::array<double, 6> yp{};
    std::array<double, 3> wb{};
    std::array<double, 6> f{};
    source->updateTime(0.0, 1.0);
    source->evaluateInternalResidual(y.data(), yp.data(), wb.data(), f.data());
    success *= isEqual(f[3], -sqrt_two * E[0] * std::cos(phi[0]), 1.0e-13);
    success *= isEqual(f[4], -sqrt_two * E[1] * std::cos(phi[1]), 1.0e-13);
    success *= isEqual(f[5], -sqrt_two * E[2] * std::cos(phi[2]), 1.0e-13);

    auto* bus = system->getBus(650);
    bus->evaluateResidual();
    source->evaluateResidual();
    success *= isEqual(bus->Ia(), source->y().getData()[0], 1.0e-13);
    success *= isEqual(bus->Ib(), source->y().getData()[1], 1.0e-13);
    success *= isEqual(bus->Ic(), source->y().getData()[2], 1.0e-13);

    auto       coupled_data = source_data;
    const auto Rs           = coupledResistance();
    const auto Ls           = coupledInductance();
    setRationalBlock(coupled_data, EMT::VoltageSourceSubmodels::Z, Rs, Ls);
    SourceT coupled_source(system->getBus(650), coupled_data);
    success *= coupled_source.allocate() == 0;
    success *= coupled_source.verify() == 0;

    const std::array<double, 6> coupled_y{0.70, -1.10, 1.40, 100.0, -200.0, 300.0};
    const std::array<double, 6> coupled_yp{-2.00, 2.30, -2.60, 0.0, 0.0, 0.0};
    const std::array<double, 3> coupled_wb{120.0, -230.0, 340.0};
    std::array<double, 6>       coupled_f{};
    coupled_source.updateTime(0.017, 1.0);
    coupled_source.evaluateInternalResidual(coupled_y.data(),
                                            coupled_yp.data(),
                                            coupled_wb.data(),
                                            coupled_f.data());
    for (std::size_t row = 0; row < 3; ++row)
    {
      double expected = coupled_wb[row] - coupled_y[row + 3];
      for (std::size_t column = 0; column < 3; ++column)
      {
        expected += Rs[row][column] * coupled_y[column]
                    + Ls[row][column] * coupled_yp[column];
      }
      success *= isEqual(coupled_f[row], expected, 2.0e-13);
    }
    const double omega = std::get<double>(
        coupled_data.parameters.at(EMT::VoltageSourceParameters::omega));
    success *= isEqual(coupled_f[3],
                       coupled_y[3] - sqrt_two * E[0] * std::cos(omega * 0.017 + phi[0]),
                       2.0e-13);
    success *= isEqual(coupled_f[4],
                       coupled_y[4] - sqrt_two * E[1] * std::cos(omega * 0.017 + phi[1]),
                       2.0e-13);
    success *= isEqual(coupled_f[5],
                       coupled_y[5] - sqrt_two * E[2] * std::cos(omega * 0.017 + phi[2]),
                       2.0e-13);

    std::array<double, 3> coupled_h{};
    coupled_source.evaluateBusResidual(coupled_y.data(), coupled_h.data());
    success *= isEqual(coupled_h[0], coupled_y[0], 1.0e-15);
    success *= isEqual(coupled_h[1], coupled_y[1], 1.0e-15);
    success *= isEqual(coupled_h[2], coupled_y[2], 1.0e-15);

    return success.report(__func__);
  }

  TestOutcome jacobianCoupling()
  {
    TestStatus success  = true;
    auto       data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);
    setRationalBlock(data.voltage_source[0],
                     EMT::VoltageSourceSubmodels::Z,
                     coupledResistance(),
                     coupledInductance());
    auto  system  = makeFixtureSystem(data);
    auto* source  = findComponent<SourceT>(*system, data);
    success      *= source != nullptr;
    if (source == nullptr)
    {
      return success.report(__func__);
    }

#ifdef GRIDKIT_ENABLE_ENZYME
    source->updateTime(0.0, 31.0);
    source->evaluateResidual();
    success *= source->evaluateJacobian() == 0;

    const auto source_row     = source->getResidualIndex(0);
    const auto bus_row        = system->getBus(650)->getResidualIndex(0);
    const auto bus_voltage    = system->getBus(650)->getVariableIndex(0);
    const auto source_current = source->getVariableIndex(0);
    const auto source_voltage = source->getVariableIndex(3);

    success *= isEqual(cooValue(*source, source_row, bus_voltage), 1.0, 1.0e-13);
    success *= isEqual(cooValue(*source, bus_row, source_current), 1.0, 1.0e-13);
    success *= isEqual(cooValue(*source,
                                source->getResidualIndex(3),
                                source_voltage),
                       1.0,
                       1.0e-13);

    success *= componentJacobianMatchesReferences(
        *system, *source, data, {13.0, 31.0});
#endif

    return success.report(__func__);
  }
} // namespace

int main()
{
  GridKit::Testing::TestingResults result;
  result += zeroInitializationAndExplicitSourceRows();
  result += jacobianCoupling();
  return result.summary();
}

#include <utility>
#include <vector>

#include <GridKit/Model/EMT/Component/Switch/SwitchImpl.hpp>
#include <GridKit/Testing/Testing.hpp>

#include "PhysicalModelTestUtils.hpp"

namespace
{
  using namespace GridKit;
  using namespace GridKit::Testing;
  using namespace GridKit::Testing::EMTTest;
  using SwitchT = EMT::Switch<double, std::size_t>;

  /// Set the open command through the source that owns the fixture's signal.
  void command(SystemT& system, double open)
  {
    system.setSignalSourceValue(0, open);
  }

  TestOutcome positionSelectsResidual()
  {
    TestStatus success = true;
    const auto data    = loadFixtureData();
    auto       system  = makeFixtureSystem(data);

    auto* device  = findComponent<SwitchT>(*system, data);
    success      *= device != nullptr;
    if (device == nullptr)
    {
      return success.report(__func__);
    }

    success *= device->size() == 3;
    success *= !device->tag()[0] && !device->tag()[1] && !device->tag()[2];

    auto* bus1             = system->getBus(632);
    auto* bus2             = system->getBus(6321);
    bus1->y().getData()[0] = 130.0;
    bus1->y().getData()[1] = -70.0;
    bus1->y().getData()[2] = -60.0;
    bus2->y().getData()[0] = 5.0;
    bus2->y().getData()[1] = -3.0;
    bus2->y().getData()[2] = -2.0;
    bus1->y().setDataUpdated();
    bus2->y().setDataUpdated();
    device->y().getData()[0] = 11.0;
    device->y().getData()[1] = -7.0;
    device->y().getData()[2] = -4.0;
    device->y().setDataUpdated();

    command(*system, 1.0);
    bus1->evaluateResidual();
    bus2->evaluateResidual();
    device->evaluateResidual();
    const auto* f = device->getResidual().getData();
    for (std::size_t phase = 0; phase < 3; ++phase)
    {
      success *= f[phase] == device->y().getData()[phase];
    }
    // The wiring is the same in both positions: -i12 at terminal 1, +i12 at
    // terminal 2.
    success *= bus1->Ia() == -11.0 && bus1->Ib() == 7.0 && bus1->Ic() == 4.0;
    success *= bus2->Ia() == 11.0 && bus2->Ib() == -7.0 && bus2->Ic() == -4.0;

    command(*system, 0.0);
    bus1->evaluateResidual();
    bus2->evaluateResidual();
    device->evaluateResidual();
    for (std::size_t phase = 0; phase < 3; ++phase)
    {
      success *= isEqual(f[phase],
                         bus2->y().getData()[phase] - bus1->y().getData()[phase],
                         1.0e-13);
    }
    success *= bus1->Ia() == -11.0 && bus2->Ia() == 11.0;

    command(*system, 0.5);
    success *= device->verify() != 0;
    command(*system, 1.0);
    success *= device->verify() == 0;

    return success.report(__func__);
  }

  TestOutcome positionKeepsFixedJacobianPattern()
  {
    TestStatus success = true;
    const auto data    = loadFixtureData();
    auto       system  = makeFixtureSystem(data);

    auto* device  = findComponent<SwitchT>(*system, data);
    success      *= device != nullptr;
    if (device == nullptr)
    {
      return success.report(__func__);
    }

    const auto capture = [&](double open)
    {
      command(*system, open);
      system->updateTime(0.0, 23.0);
      system->evaluateResidual();
      system->evaluateJacobian();
      return cooPattern(*device);
    };

    const auto opened        = capture(1.0);
    const auto closed        = capture(0.0);
    const auto opened_again  = capture(1.0);
    success                 *= opened.size() == 15;
    success                 *= opened == closed && closed == opened_again;

    // Reserved entries are present with zero values in the inactive position.
    command(*system, 1.0);
    system->updateTime(0.0, 23.0);
    system->evaluateResidual();
    system->evaluateJacobian();
    auto* bus1 = system->getBus(632);
    auto* bus2 = system->getBus(6321);
    for (std::size_t phase = 0; phase < 3; ++phase)
    {
      success *= cooValue(*device,
                          device->getResidualIndex(phase),
                          device->getVariableIndex(phase))
                 == 1.0;
      success *= cooValue(*device,
                          device->getResidualIndex(phase),
                          bus1->getVariableIndex(phase))
                 == 0.0;
      success *= cooValue(*device,
                          device->getResidualIndex(phase),
                          bus2->getVariableIndex(phase))
                 == 0.0;
      success *= cooValue(*device,
                          bus1->getResidualIndex(phase),
                          device->getVariableIndex(phase))
                 == -1.0;
      success *= cooValue(*device,
                          bus2->getResidualIndex(phase),
                          device->getVariableIndex(phase))
                 == 1.0;
    }

    command(*system, 0.0);
    system->updateTime(0.0, 23.0);
    system->evaluateResidual();
    system->evaluateJacobian();
    for (std::size_t phase = 0; phase < 3; ++phase)
    {
      success *= cooValue(*device,
                          device->getResidualIndex(phase),
                          device->getVariableIndex(phase))
                 == 0.0;
      success *= cooValue(*device,
                          device->getResidualIndex(phase),
                          bus1->getVariableIndex(phase))
                 == -1.0;
      success *= cooValue(*device,
                          device->getResidualIndex(phase),
                          bus2->getVariableIndex(phase))
                 == 1.0;
    }

    // The reference systems are built from data, so the commanded position has
    // to be carried in the constant source that owns the open signal.
    auto closed_data = data;
    closed_data.constant_source[0]
        .parameters[PhasorDynamics::ConstantSignalSourceParameters::Sr] = 0.0;
    command(*system, 0.0);
    success *= componentJacobianMatchesReferences(
        *system, *device, closed_data, {7.0, 23.0});

    command(*system, 1.0);
    success *= componentJacobianMatchesReferences(
        *system, *device, data, {7.0, 23.0});

    return success.report(__func__);
  }

  TestOutcome terminalBusCarriesNoVoltageCoefficient()
  {
    TestStatus success = true;
    const auto data    = loadFixtureData();
    auto       system  = makeFixtureSystem(data);

    auto* device  = findComponent<SwitchT>(*system, data);
    success      *= device != nullptr;
    if (device == nullptr)
    {
      return success.report(__func__);
    }

    std::vector<EMT::BusVoltageContribution<double, std::size_t>> contributions;
    device->appendBusVoltageContributions(contributions);
    success *= contributions.size() == 2;
    for (const auto& contribution : contributions)
    {
      success *= !contribution.differentiable_injection;
      success *= EMT::Detail::matrixRank(contribution.algebraic_block) == 0;
      success *= EMT::Detail::matrixRank(contribution.derivative_block) == 0;
    }

    // The terminal bus is algebraic and never uses a differentiated current
    // balance, so the switch is free of an alpha-scaled injection.
    success *= system->getBus(6321)->voltageClass()
               == EMT::BusVoltageClass::algebraic;
    success *= !system->getBus(6321)->differentiatedKCL();

    return success.report(__func__);
  }
} // namespace

int main()
{
  GridKit::Testing::TestingResults result;
  result += positionSelectsResidual();
  result += positionKeepsFixedJacobianPattern();
  result += terminalBusCarriesNoVoltageCoefficient();
  return result.summary();
}

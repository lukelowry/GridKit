#include <cmath>

#include <GridKit/Testing/Testing.hpp>

#include "PhysicalModelTestUtils.hpp"

namespace
{
  using namespace GridKit;
  using namespace GridKit::Testing;
  using namespace GridKit::Testing::EMTTest;

  TestOutcome abcInitializationAndKclReset()
  {
    TestStatus success  = true;
    const auto data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);
    auto system         = makeFixtureSystem(data);

    const double sqrt_two = std::sqrt(2.0);
    for (const auto& bus_data : data.bus)
    {
      auto* bus  = system->getBus(bus_data.bus_id);
      success   *= bus->size() == 3;
      success   *= bus->busID() == bus_data.bus_id;
      success   *= isEqual(bus->Va(), sqrt_two * bus_data.Va0.real(), 1.0e-13);
      success   *= isEqual(bus->Vb(), sqrt_two * bus_data.Vb0.real(), 1.0e-13);
      success   *= isEqual(bus->Vc(), sqrt_two * bus_data.Vc0.real(), 1.0e-13);
      success   *= isEqual(bus->Vap(), -sqrt_two * data.omega0 * bus_data.Va0.imag(), 1.0e-13);
      success   *= isEqual(bus->Vbp(), -sqrt_two * data.omega0 * bus_data.Vb0.imag(), 1.0e-13);
      success   *= isEqual(bus->Vcp(), -sqrt_two * data.omega0 * bus_data.Vc0.imag(), 1.0e-13);

      success *= bus->tag()[0] && bus->tag()[1] && bus->tag()[2];

      bus->evaluateResidual();
      bus->Ia() += 1.25;
      bus->Ib() += -2.50;
      bus->Ic() += 3.75;
      bus->Ia() += -0.50;
      bus->Ib() += 1.00;
      bus->Ic() += -1.50;
      bus->Ia() += 0.125;
      bus->Ib() += 0.250;
      bus->Ic() += 0.375;
      success   *= isEqual(bus->Ia(), 0.875, 1.0e-15);
      success   *= isEqual(bus->Ib(), -1.250, 1.0e-15);
      success   *= isEqual(bus->Ic(), 2.625, 1.0e-15);

      success *= bus->evaluateResidual() == 0;
      success *= bus->Ia() == 0.0;
      success *= bus->Ib() == 0.0;
      success *= bus->Ic() == 0.0;
    }

    return success.report(__func__);
  }

  TestOutcome fixedJacobianStructure()
  {
    TestStatus success  = true;
    const auto data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);
    auto system         = makeFixtureSystem(data);

    for (const auto& bus_data : data.bus)
    {
      auto* bus       = system->getBus(bus_data.bus_id);
      success        *= bus->evaluateJacobian() == 0;
      auto* jacobian  = bus->getCooJacobian();
      success        *= jacobian != nullptr;
      if (jacobian != nullptr)
      {
        success *= jacobian->getNnz() == 9;
        for (std::size_t entry = 0; entry < jacobian->getNnz(); ++entry)
        {
          success *= jacobian->getValues()[entry] == 0.0;
        }
      }

      success *= componentJacobianMatchesReferences(
          *system, *bus, data, {3.0, 37.0}, false);
    }

    return success.report(__func__);
  }
} // namespace

int main()
{
  GridKit::Testing::TestingResults result;
  result += abcInitializationAndKclReset();
  result += fixedJacobianStructure();
  return result.summary();
}

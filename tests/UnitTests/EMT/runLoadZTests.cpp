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

  EMT::ABCMatrix<double> zeroMatrix()
  {
    return {{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};
  }

  TestOutcome zeroInitializationAndBusInjection()
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

    // The fixture's fault branch is purely resistive and sits behind a switch,
    // so its current is algebraic while the feeder load stays differential.
    success *= normal_load->tag()[0] && normal_load->tag()[1]
               && normal_load->tag()[2];
    success *= !fault_load->tag()[0] && !fault_load->tag()[1]
               && !fault_load->tag()[2];

    std::vector<EMT::InitialStateVariable> differential_layout;
    std::vector<EMT::InitialStateVariable> algebraic_layout;
    normal_load->appendInitialStateVariables(differential_layout);
    fault_load->appendInitialStateVariables(algebraic_layout);
    success *= differential_layout.size() == 1
               && differential_layout[0].name == "i";
    success *= algebraic_layout.empty();

    auto* bus                     = system->getBus(670);
    normal_load->y().getData()[0] = 0.70;
    normal_load->y().getData()[1] = -1.10;
    normal_load->y().getData()[2] = 1.40;
    normal_load->y().setDataUpdated();

    bus->evaluateResidual();
    normal_load->evaluateResidual();
    success *= isEqual(bus->Ia(), 0.70, 1.0e-13);
    success *= isEqual(bus->Ib(), -1.10, 1.0e-13);
    success *= isEqual(bus->Ic(), 1.40, 1.0e-13);

    auto       coupled_data = data.loadz[0];
    const auto R            = coupledResistance();
    const auto L            = coupledInductance();
    setRationalBlock(coupled_data, EMT::LoadZSubmodels::Z, R, L);
    LoadT coupled_load(system->getBus(670), coupled_data);
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
    coupled_load.evaluateBusResidual(y.data(), h.data());
    success *= h[0] == y[0] && h[1] == y[1] && h[2] == y[2];

    return success.report(__func__);
  }

  TestOutcome algebraicCurrentRequiresInvertibleResistance()
  {
    TestStatus success = true;
    const auto data    = loadFixtureData();
    auto       system  = makeFixtureSystem(data);

    auto       algebraic_data = data.loadz[0];
    const auto R              = coupledResistance();
    setRationalBlock(algebraic_data, EMT::LoadZSubmodels::Z, R, zeroMatrix());
    LoadT algebraic_load(system->getBus(670), algebraic_data);
    success *= algebraic_load.allocate() == 0;
    success *= algebraic_load.verify() == 0;
    success *= algebraic_load.tagDifferentiable() == 0;
    success *= !algebraic_load.tag()[0] && !algebraic_load.tag()[1]
               && !algebraic_load.tag()[2];

    // A zero linear coefficient makes the branch conductance the bus-voltage
    // coefficient, which is what keeps the terminal bus nonsingular.
    std::vector<EMT::BusVoltageContribution<double, std::size_t>> contributions;
    algebraic_load.appendBusVoltageContributions(contributions);
    success *= contributions.size() == 1;
    if (contributions.size() == 1)
    {
      success *= !contributions[0].differentiable_injection;
      success *= EMT::Detail::matrixRank(contributions[0].algebraic_block) == 3;
      success *= EMT::Detail::matrixRank(contributions[0].derivative_block) == 0;
    }

    auto singular_data = data.loadz[0];
    setRationalBlock(singular_data, EMT::LoadZSubmodels::Z, zeroMatrix(), zeroMatrix());
    LoadT singular_load(system->getBus(670), singular_data);
    success *= singular_load.allocate() == 0;
    success *= singular_load.verify() != 0;

    auto partial_data        = data.loadz[0];
    auto partial_inductance  = coupledInductance();
    partial_inductance[2]    = {0.0, 0.0, 0.0};
    partial_inductance[0][2] = 0.0;
    partial_inductance[1][2] = 0.0;
    setRationalBlock(partial_data, EMT::LoadZSubmodels::Z, R, partial_inductance);
    LoadT partial_load(system->getBus(670), partial_data);
    success *= partial_load.allocate() == 0;
    success *= partial_load.verify() != 0;

    // A differential branch contributes no bus-voltage coefficient at all.
    auto differential_data = data.loadz[0];
    setRationalBlock(differential_data,
                     EMT::LoadZSubmodels::Z,
                     R,
                     coupledInductance());
    LoadT differential_load(system->getBus(670), differential_data);
    success *= differential_load.allocate() == 0;
    success *= differential_load.verify() == 0;
    contributions.clear();
    differential_load.appendBusVoltageContributions(contributions);
    success *= contributions.size() == 1;
    if (contributions.size() == 1)
    {
      success *= contributions[0].differentiable_injection;
      success *= EMT::Detail::matrixRank(contributions[0].algebraic_block) == 0;
    }

    return success.report(__func__);
  }

  TestOutcome jacobianMatchesReferences()
  {
    TestStatus success  = true;
    auto       data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);
    setRationalBlock(data.loadz[0],
                     EMT::LoadZSubmodels::Z,
                     coupledResistance(),
                     coupledInductance());
    auto  system       = makeFixtureSystem(data);
    auto* normal_load  = findComponent<LoadT>(*system, data, 0);
    auto* fault_load   = findComponent<LoadT>(*system, data, 1);
    success           *= normal_load != nullptr;
    success           *= fault_load != nullptr;
    if (normal_load == nullptr || fault_load == nullptr)
    {
      return success.report(__func__);
    }

    success *= componentJacobianMatchesReferences(
        *system, *normal_load, data, {7.0, 23.0});
    success *= componentJacobianMatchesReferences(
        *system, *fault_load, data, {7.0, 23.0});

    return success.report(__func__);
  }
} // namespace

int main()
{
  GridKit::Testing::TestingResults result;
  result += zeroInitializationAndBusInjection();
  result += algebraicCurrentRequiresInvertibleResistance();
  result += jacobianMatchesReferences();
  return result.summary();
}

#include <array>

#include <GridKit/Model/EMT/Component/Line/LineLumped/LineLumpedImpl.hpp>
#include <GridKit/Testing/Testing.hpp>

#include "PhysicalModelTestUtils.hpp"

namespace
{
  using namespace GridKit;
  using namespace GridKit::Testing;
  using namespace GridKit::Testing::EMTTest;
  using LineT = EMT::LineLumped<double, std::size_t>;

  TestOutcome zeroInitializationAndExplicitMutualTerms()
  {
    TestStatus success  = true;
    const auto data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);
    auto system         = makeFixtureSystem(data);

    auto* line1  = findComponent<LineT>(*system, data, 0);
    auto* line2  = findComponent<LineT>(*system, data, 1);
    success     *= line1 != nullptr && line2 != nullptr;
    if (line1 == nullptr || line2 == nullptr)
    {
      return success.report(__func__);
    }

    success *= line1->size() == 9 && line2->size() == 9;
    for (std::size_t index = 0; index < line1->size(); ++index)
    {
      line1->y().getData()[index]  = 1.0;
      line1->yp().getData()[index] = -1.0;
      line2->y().getData()[index]  = 2.0;
      line2->yp().getData()[index] = -2.0;
    }
    success *= line1->initialize() == 0;
    success *= line2->initialize() == 0;
    for (std::size_t index = 0; index < line1->size(); ++index)
    {
      success *= line1->y().getData()[index] == 0.0;
      success *= line1->yp().getData()[index] == 0.0;
      success *= line2->y().getData()[index] == 0.0;
      success *= line2->yp().getData()[index] == 0.0;
    }
    success *= line1->tag()[0] && line1->tag()[1] && line1->tag()[2];
    success *= !line1->tag()[3] && !line1->tag()[8];

    const auto&  line_data = data.line_lumped[0];
    const double dx        = std::get<double>(
        line_data.parameters.at(EMT::LineLumpedParameters::dx));
    const auto  series = rationalBlock(line_data, EMT::LineLumpedSubmodels::Zp);
    const auto  shunt  = rationalBlock(line_data, EMT::LineLumpedSubmodels::Yp);
    const auto& Rp     = series.D;
    const auto& Lp     = series.E;
    const auto& Gp     = shunt.D;
    const auto& Cp     = shunt.E;

    std::array<double, 9> y{0.40, -0.75, 1.10, -0.20, 0.35, -0.55, 0.65, -0.80, 0.95};
    std::array<double, 9> yp{1.20, -1.40, 1.60, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 6> wb{10.0, -20.0, 30.0, -40.0, 50.0, -60.0};
    std::array<double, 6> wbp{70.0, -80.0, 90.0, -100.0, 110.0, -120.0};
    std::array<double, 9> f{};
    line1->evaluateInternalResidual(y.data(), yp.data(), wb.data(), wbp.data(), f.data());

    std::array<double, 9> expected{};
    for (std::size_t row = 0; row < 3; ++row)
    {
      expected[row]     = wb[row + 3] - wb[row];
      expected[row + 3] = 2.0 * y[row + 3];
      expected[row + 6] = 2.0 * y[row + 6];
      for (std::size_t column = 0; column < 3; ++column)
      {
        expected[row] += dx * Rp[row][column] * y[column]
                         + dx * Lp[row][column] * yp[column];
        expected[row + 3] += dx * Gp[row][column] * wb[column]
                             + dx * Cp[row][column] * wbp[column];
        expected[row + 6] += dx * Gp[row][column] * wb[column + 3]
                             + dx * Cp[row][column] * wbp[column + 3];
      }
    }
    for (std::size_t row = 0; row < expected.size(); ++row)
    {
      success *= isEqual(f[row], expected[row], 2.0e-13);
    }

    std::array<double, 3> h1{};
    std::array<double, 3> h2{};
    line1->evaluateBus1Residual(y.data(), h1.data());
    line1->evaluateBus2Residual(y.data(), h2.data());
    success *= isEqual(h1[0], y[3] - y[0], 1.0e-15);
    success *= isEqual(h1[1], y[4] - y[1], 1.0e-15);
    success *= isEqual(h1[2], y[5] - y[2], 1.0e-15);
    success *= isEqual(h2[0], y[6] + y[0], 1.0e-15);
    success *= isEqual(h2[1], y[7] + y[1], 1.0e-15);
    success *= isEqual(h2[2], y[8] + y[2], 1.0e-15);

    return success.report(__func__);
  }

  TestOutcome derivativeBusJacobian()
  {
    TestStatus success  = true;
    const auto data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);
    auto  system        = makeFixtureSystem(data);
    auto* line          = findComponent<LineT>(*system, data, 0);
    success            *= line != nullptr;
    if (line == nullptr)
    {
      return success.report(__func__);
    }

#ifdef GRIDKIT_ENABLE_ENZYME
    const auto&  line_data = data.line_lumped[0];
    const double dx        = std::get<double>(
        line_data.parameters.at(EMT::LineLumpedParameters::dx));
    const auto& Cp =
        rationalBlock(line_data, EMT::LineLumpedSubmodels::Yp).E;

    const auto       row    = line->getResidualIndex(3);
    const auto       column = system->getBus(650)->getVariableIndex(1);
    constexpr double alpha1 = 11.0;
    constexpr double alpha2 = 29.0;

    line->updateTime(0.0, alpha1);
    line->evaluateResidual();
    success             *= line->evaluateJacobian() == 0;
    const double value1  = cooValue(*line, row, column);

    line->updateTime(0.0, alpha2);
    line->evaluateResidual();
    success             *= line->evaluateJacobian() == 0;
    const double value2  = cooValue(*line, row, column);

    success *= isEqual(value2 - value1,
                       (alpha2 - alpha1) * dx * Cp[0][1],
                       1.0e-10);

    success *= componentJacobianMatchesReferences(
        *system, *line, data, {alpha1, alpha2});
    auto* second_line  = findComponent<LineT>(*system, data, 1);
    success           *= second_line != nullptr;
    if (second_line != nullptr)
    {
      success *= componentJacobianMatchesReferences(
          *system, *second_line, data, {alpha1, alpha2});
    }
#endif

    return success.report(__func__);
  }
} // namespace

int main()
{
  GridKit::Testing::TestingResults result;
  result += zeroInitializationAndExplicitMutualTerms();
  result += derivativeBusJacobian();
  return result.summary();
}

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <vector>

#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Testing/Testing.hpp>

#include "EMTTestFixture.hpp"

namespace
{
  using namespace GridKit;
  using namespace GridKit::Testing;
  using namespace GridKit::Testing::EMTTest;
  using VectorFitT = EMT::VectorFit<double, std::size_t>;
  using SignalT    = PhasorDynamics::SignalNode<double, std::size_t>;

  struct SignalHarness
  {
    SignalT input_a;
    SignalT input_b;
    SignalT input_c;
    SignalT output_a;
    SignalT output_b;
    SignalT output_c;

    double      ua{2.0};
    double      ub{-1.0};
    double      uc{0.5};
    double      uap{-4.0};
    double      ubp{3.0};
    double      ucp{1.0};
    std::size_t index_a{100};
    std::size_t index_b{101};
    std::size_t index_c{102};

    void connect(VectorFitT& model, bool link_derivatives = true)
    {
      if (link_derivatives)
      {
        input_a.set(&ua, &uap, &index_a);
        input_b.set(&ub, &ubp, &index_b);
        input_c.set(&uc, &ucp, &index_c);
      }
      else
      {
        input_a.set(&ua, &index_a);
        input_b.set(&ub, &index_b);
        input_c.set(&uc, &index_c);
      }
      model.getSignals()
          .attachSignalNode<EMT::VectorFitExternalVariables::input_a>(&input_a);
      model.getSignals()
          .attachSignalNode<EMT::VectorFitExternalVariables::input_b>(&input_b);
      model.getSignals()
          .attachSignalNode<EMT::VectorFitExternalVariables::input_c>(&input_c);
      model.getSignals()
          .assignSignalNode<EMT::VectorFitInternalVariables::out_a>(&output_a);
      model.getSignals()
          .assignSignalNode<EMT::VectorFitInternalVariables::out_b>(&output_b);
      model.getSignals()
          .assignSignalNode<EMT::VectorFitInternalVariables::out_c>(&output_c);
    }
  };

  bool initializesLocalStateToZero(VectorFitT& model)
  {
    for (std::size_t index = 0; index < model.size(); ++index)
    {
      model.y().getData()[index]  = 1.0;
      model.yp().getData()[index] = -1.0;
    }
    if (model.initialize() != 0)
    {
      return false;
    }
    for (std::size_t index = 0; index < model.size(); ++index)
    {
      if (model.y().getData()[index] != 0.0
          || model.yp().getData()[index] != 0.0)
      {
        return false;
      }
    }
    return true;
  }

  EMT::VectorFitData<double, std::size_t> runtimePoleData()
  {
    EMT::VectorFitData<double, std::size_t> data;
    data.device_class          = "VectorFit";
    data.disambiguation_string = "runtime_poles";

    EMT::ABCMatrix<double> D{{{{1.0, 0.2, -0.1}},
                              {{0.3, 0.8, 0.1}},
                              {{-0.2, 0.4, 1.2}}}};
    EMT::ABCMatrix<double> E{{{{0.01, 0.002, 0.0}},
                              {{0.0, 0.02, 0.001}},
                              {{0.003, 0.0, 0.015}}}};
    data.parameters[EMT::VectorFitParameters::D]     = D;
    data.parameters[EMT::VectorFitParameters::E]     = E;
    data.parameters[EMT::VectorFitParameters::poles] = std::vector<std::complex<double>>{
        {-10.0, 0.0}, {-20.0, 30.0}, {-20.0, -30.0}};

    EMT::ABCMatrix<std::complex<double>> real_residue{};
    real_residue[0] = {{{2.0, 0.0}, {0.1, 0.0}, {-0.2, 0.0}}};
    real_residue[1] = {{{0.3, 0.0}, {1.5, 0.0}, {0.4, 0.0}}};
    real_residue[2] = {{{-0.1, 0.0}, {0.2, 0.0}, {1.0, 0.0}}};

    EMT::ABCMatrix<std::complex<double>> complex_residue{};
    complex_residue[0] = {{{0.5, 0.2}, {0.1, -0.1}, {0.2, 0.3}}};
    complex_residue[1] = {{{-0.2, 0.1}, {0.8, 0.4}, {0.1, -0.2}}};
    complex_residue[2] = {{{0.3, -0.1}, {-0.1, 0.2}, {0.6, 0.5}}};

    EMT::ABCMatrix<std::complex<double>> conjugate_residue{};
    for (std::size_t row = 0; row < 3; ++row)
    {
      for (std::size_t column = 0; column < 3; ++column)
      {
        conjugate_residue[row][column] = std::conj(complex_residue[row][column]);
      }
    }

    data.parameters[EMT::VectorFitParameters::residues] = std::vector<EMT::ABCMatrix<std::complex<double>>>{
        real_residue, complex_residue, conjugate_residue};
    return data;
  }

  bool jacobianMatchesFiniteDifference(VectorFitT&    model,
                                       SignalHarness& signals,
                                       double         alpha)
  {
    model.updateTime(0.0, alpha);
    if (model.evaluateResidual() != 0 || model.evaluateJacobian() != 0
        || model.nnz() != 60)
    {
      return false;
    }

    auto* jacobian = model.getCooJacobian();
    if (jacobian == nullptr)
    {
      return false;
    }

    std::vector<std::vector<double>> dense(
        model.size(), std::vector<double>(103, 0.0));
    for (std::size_t entry = 0; entry < jacobian->getNnz(); ++entry)
    {
      dense[jacobian->getRowData()[entry]][jacobian->getColData()[entry]] += jacobian->getValues()[entry];
    }

    constexpr double epsilon = 1.0e-7;
    for (std::size_t column = 0; column < model.size(); ++column)
    {
      model.y().getData()[column]  += epsilon;
      model.yp().getData()[column] += alpha * epsilon;
      model.evaluateResidual();
      std::vector<double> plus(model.size());
      for (std::size_t row = 0; row < model.size(); ++row)
      {
        plus[row] = model.getResidual().getData()[row];
      }

      model.y().getData()[column]  -= 2.0 * epsilon;
      model.yp().getData()[column] -= 2.0 * alpha * epsilon;
      model.evaluateResidual();
      for (std::size_t row = 0; row < model.size(); ++row)
      {
        const double finite_difference = (plus[row] - model.getResidual().getData()[row])
                                         / (2.0 * epsilon);
        if (!isEqual(finite_difference, dense[row][column], 1.0e-6))
        {
          return false;
        }
      }

      model.y().getData()[column]  += epsilon;
      model.yp().getData()[column] += alpha * epsilon;
    }

    std::array<double*, 3>     values{&signals.ua, &signals.ub, &signals.uc};
    std::array<double*, 3>     derivatives{&signals.uap, &signals.ubp, &signals.ucp};
    std::array<std::size_t, 3> columns{100, 101, 102};
    for (std::size_t input = 0; input < 3; ++input)
    {
      *values[input]      += epsilon;
      *derivatives[input] += alpha * epsilon;
      model.evaluateResidual();
      std::vector<double> plus(model.size());
      for (std::size_t row = 0; row < model.size(); ++row)
      {
        plus[row] = model.getResidual().getData()[row];
      }

      *values[input]      -= 2.0 * epsilon;
      *derivatives[input] -= 2.0 * alpha * epsilon;
      model.evaluateResidual();
      for (std::size_t row = 0; row < model.size(); ++row)
      {
        const double finite_difference = (plus[row] - model.getResidual().getData()[row])
                                         / (2.0 * epsilon);
        if (!isEqual(finite_difference,
                     dense[row][columns[input]],
                     1.0e-6))
        {
          return false;
        }
      }

      *values[input]      += epsilon;
      *derivatives[input] += alpha * epsilon;
    }
    return true;
  }

  bool verificationFails(
      const EMT::VectorFitData<double, std::size_t>& data,
      bool                                           link_derivatives = true)
  {
    VectorFitT    model(data);
    SignalHarness signals;
    signals.connect(model, link_derivatives);
    return model.allocate() == 0 && model.verify() != 0;
  }

  TestOutcome fixtureQZeroOperator()
  {
    TestStatus success  = true;
    const auto data     = loadFixtureData();
    success            *= isThreeBusMutuallyCoupled(data);
    auto  system        = makeFixtureSystem(data);
    auto* model         = findComponent<VectorFitT>(*system, data);
    success            *= model != nullptr;
    if (model == nullptr)
    {
      return success.report(__func__);
    }

    success *= model->size() == 3;
    success *= initializesLocalStateToZero(*model);
    success *= !model->tag()[0] && !model->tag()[1] && !model->tag()[2];
    success *= system->getSignal(5)->read() == 0.0;
    success *= system->getSignal(6)->read() == 0.0;
    success *= system->getSignal(7)->read() == 0.0;
    success *= system->getSignal(5)->derivativeLinked();
    success *= system->getSignal(5)->readDerivative() == 0.0;

    return success.report(__func__);
  }

  TestOutcome runtimePolesAndAnalyticJacobian()
  {
    TestStatus    success = true;
    auto          data    = runtimePoleData();
    VectorFitT    model(data);
    SignalHarness signals;
    signals.connect(model);

    success *= model.allocate() == 0;
    success *= model.verify() == 0;
    success *= model.tagDifferentiable() == 0;
    success *= initializesLocalStateToZero(model);
    model.updateTime(0.0, 7.0);
    success *= model.evaluateResidual() == 0;
    success *= model.size() == 12;
    for (std::size_t state = 0; state < 9; ++state)
    {
      success *= model.tag()[state];
    }
    success *= !model.tag()[9] && !model.tag()[10] && !model.tag()[11];
    success *= signals.output_a.linked();
    success *= signals.output_a.derivativeLinked();

    std::vector<EMT::InitialStateVariable> initial_state_layout;
    model.appendInitialStateVariables(initial_state_layout);
    success *= initial_state_layout.size() == 3;
    if (initial_state_layout.size() == 3)
    {
      success *= initial_state_layout[0].name == "w";
      success *= initial_state_layout[0].index == 0;
      success *= initial_state_layout[0].local_offset == 0;
      success *= initial_state_layout[1].name == "w";
      success *= initial_state_layout[1].index == 1;
      success *= initial_state_layout[1].local_offset == 3;
      success *= initial_state_layout[2].name == "v";
      success *= initial_state_layout[2].index == 1;
      success *= initial_state_layout[2].local_offset == 6;
    }

    success *= jacobianMatchesFiniteDifference(model, signals, 7.0);
    success *= jacobianMatchesFiniteDifference(model, signals, 29.0);

    return success.report(__func__);
  }

  TestOutcome validationRules()
  {
    TestStatus success       = true;
    const auto fixture_data  = loadFixtureData();
    success                 *= isThreeBusMutuallyCoupled(fixture_data);

    auto bad_pole_conjugate = runtimePoleData();
    auto poles              = std::get<std::vector<std::complex<double>>>(
        bad_pole_conjugate.parameters.at(EMT::VectorFitParameters::poles));
    poles[2]                                                        = {-21.0, -30.0};
    bad_pole_conjugate.parameters[EMT::VectorFitParameters::poles]  = poles;
    success                                                        *= verificationFails(bad_pole_conjugate);

    auto bad_count = runtimePoleData();
    auto residues  = std::get<
         std::vector<EMT::ABCMatrix<std::complex<double>>>>(
        bad_count.parameters.at(EMT::VectorFitParameters::residues));
    residues.pop_back();
    bad_count.parameters[EMT::VectorFitParameters::residues]  = residues;
    success                                                  *= verificationFails(bad_count);

    auto lone_complex = runtimePoleData();
    poles             = std::get<std::vector<std::complex<double>>>(
        lone_complex.parameters.at(EMT::VectorFitParameters::poles));
    residues = std::get<std::vector<EMT::ABCMatrix<std::complex<double>>>>(
        lone_complex.parameters.at(EMT::VectorFitParameters::residues));
    poles.resize(1);
    residues.resize(1);
    poles[0]                                                     = {-20.0, 30.0};
    lone_complex.parameters[EMT::VectorFitParameters::poles]     = poles;
    lone_complex.parameters[EMT::VectorFitParameters::residues]  = residues;
    success                                                     *= verificationFails(lone_complex);

    auto bad_residue_conjugate = runtimePoleData();
    residues                   = std::get<std::vector<EMT::ABCMatrix<std::complex<double>>>>(
        bad_residue_conjugate.parameters.at(
            EMT::VectorFitParameters::residues));
    residues[2][0][0]                                                     = {0.5, -0.1};
    bad_residue_conjugate.parameters[EMT::VectorFitParameters::residues]  = residues;
    success                                                              *= verificationFails(bad_residue_conjugate);

    auto complex_residue_on_real_pole = runtimePoleData();
    poles                             = std::get<std::vector<std::complex<double>>>(
        complex_residue_on_real_pole.parameters.at(
            EMT::VectorFitParameters::poles));
    residues = std::get<std::vector<EMT::ABCMatrix<std::complex<double>>>>(
        complex_residue_on_real_pole.parameters.at(
            EMT::VectorFitParameters::residues));
    poles.resize(1);
    residues.resize(1);
    residues[0][0][0]                                                            = {2.0, 0.25};
    complex_residue_on_real_pole.parameters[EMT::VectorFitParameters::poles]     = poles;
    complex_residue_on_real_pole.parameters[EMT::VectorFitParameters::residues]  = residues;
    success                                                                     *= verificationFails(complex_residue_on_real_pole);

    success *= verificationFails(runtimePoleData(), false);

    auto q_zero_with_E = fixture_data.vector_fit[0];
    auto E             = std::get<EMT::ABCMatrix<double>>(
        q_zero_with_E.parameters.at(EMT::VectorFitParameters::E));
    E[0][0]                                                = 0.1;
    q_zero_with_E.parameters[EMT::VectorFitParameters::E]  = E;
    success                                               *= verificationFails(q_zero_with_E, false);

    auto          q_zero_without_E = fixture_data.vector_fit[0];
    VectorFitT    algebraic(q_zero_without_E);
    SignalHarness derivative_free_signals;
    derivative_free_signals.connect(algebraic, false);
    success *= algebraic.allocate() == 0;
    success *= algebraic.verify() == 0;
    success *= initializesLocalStateToZero(algebraic);
    algebraic.updateTime(0.0, 13.0);
    success *= algebraic.evaluateResidual() == 0;
    success *= !derivative_free_signals.input_a.derivativeLinked();

    return success.report(__func__);
  }
} // namespace

int main()
{
  GridKit::Testing::TestingResults result;
  result += fixtureQZeroOperator();
  result += runtimePolesAndAnalyticJacobian();
  result += validationRules();
  return result.summary();
}

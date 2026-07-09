#pragma once

#include <array>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <GridKit/AutomaticDifferentiation/DependencyTracking/Variable.hpp>
#include <GridKit/Definitions.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Model/PhasorDynamics/Stabilizer/IEEEST/Ieeest.hpp>
#include <GridKit/Model/PhasorDynamics/Stabilizer/IEEEST/IeeestData.hpp>
#include <GridKit/Model/PhasorDynamics/Stabilizer/StabilizerFactory.hpp>
#include <GridKit/Testing/TestHelpers.hpp>
#include <GridKit/Testing/Testing.hpp>
#include <GridKit/Utilities/MapFromCsr.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class StabilizerIeeestTests
    {
    public:
      using RealT = typename PhasorDynamics::Component<ScalarT, IdxT>::RealT;
      using DataT = PhasorDynamics::Stabilizer::IeeestData<RealT, IdxT>;

      template <size_t order>
      using InternalVariables = PhasorDynamics::Stabilizer::IeeestInternalVariables<order>;

      StabilizerIeeestTests()  = default;
      ~StabilizerIeeestTests() = default;

      TestOutcome constructor()
      {
        TestStatus success = true;

        {
          // Zeroth order: no notch states, only X5..X7 and V4..VSS.
          PhasorDynamics::Stabilizer::Ieeest<ScalarT, IdxT, 0> model;
          success *= (model.size() == 8);
          success *= (model.getMonitor() == nullptr);
        }

        success *= checkConstructedSize<0>();
        success *= checkConstructedSize<1>();
        success *= checkConstructedSize<2>();
        success *= checkConstructedSize<3>();
        success *= checkConstructedSize<4>();

        return success.report(__func__);
      }

      template <size_t order>
      TestOutcome init()
      {
        TestStatus success = true;

        using Params = PhasorDynamics::Stabilizer::IeeestParameters;

        struct InitCase
        {
          RealT   T6;
          RealT   Ks;
          ScalarT u;
          RealT   Lsmin;
          RealT   Lsmax;
          ScalarT expected_v7;
          ScalarT expected_vss;
        };

        // The smooth clamp only approximates the hard limits, hence the
        // looser tolerance on the limited output.
        const auto                  loose_tol = static_cast<RealT>(1.0e-4);
        const std::vector<InitCase> cases     = {
            {0.0, 0.0, 0.25, -1.0, 1.0, 0.0, 0.0},
            {0.0, 4.0, 0.25, 0.2, 0.6, 0.0, 0.2},
            {5.0, 3.0, 0.25, -1.0, 1.0, 0.0, 0.0},
        };

        const auto V7  = static_cast<size_t>(InternalVariables<order>::V7);
        const auto VSS = static_cast<size_t>(InternalVariables<order>::VSS);

        for (const auto& test : cases)
        {
          PhasorDynamics::SignalNode<ScalarT, IdxT> u_node;
          PhasorDynamics::SignalNode<ScalarT, IdxT> vss_node;
          ScalarT                                   u_value{test.u};
          IdxT                                      u_index{12};
          ScalarT                                   vss_value{0.0};
          IdxT                                      vss_index{INVALID_INDEX<IdxT>};

          u_node.set(&u_value, &u_index);
          vss_node.set(&vss_value, &vss_index);

          auto data                      = makeOrderData<order>();
          data.parameters[Params::T6]    = test.T6;
          data.parameters[Params::Ks]    = test.Ks;
          data.parameters[Params::Lsmin] = test.Lsmin;
          data.parameters[Params::Lsmax] = test.Lsmax;

          PhasorDynamics::Stabilizer::Ieeest<ScalarT, IdxT, order> model(data);
          model.getSignals().template attachSignalNode<PhasorDynamics::Stabilizer::IeeestExternalVariables::U>(&u_node);
          model.getSignals().template assignSignalNode<InternalVariables<order>::VSS>(&vss_node);

          model.allocate();
          success *= (model.verify() == 0);
          model.initialize();

          success       *= vss_node.linked();
          success       *= (vss_node.getVariableIndex() == static_cast<IdxT>(VSS));
          const auto* y  = model.y().getData();
          success       *= isEqual(y[V7], test.expected_v7, tol_);
          success       *= isEqual(y[VSS], test.expected_vss, loose_tol);
          success       *= isEqual(vss_node.read(), test.expected_vss, loose_tol);
        }

        const std::string name = orderedName(__func__, order);
        return success.report(name.c_str());
      }

      template <size_t order>
      TestOutcome zeroInitialResidual()
      {
        TestStatus success = true;

        using Params = PhasorDynamics::Stabilizer::IeeestParameters;

        // The second case exercises the TIME_CONSTANT_MINIMUM flooring.
        const std::vector<std::array<RealT, 3>> time_constant_cases = {
            {1.0, 1.0, 5.0},
            {0.0, 0.0, 0.0},
        };

        for (const auto& T : time_constant_cases)
        {
          PhasorDynamics::SignalNode<ScalarT, IdxT> u_node;
          PhasorDynamics::SignalNode<ScalarT, IdxT> vss_node;
          ScalarT                                   u_value{0.25};
          IdxT                                      u_index{12};
          ScalarT                                   vss_value{0.0};
          IdxT                                      vss_index{INVALID_INDEX<IdxT>};

          u_node.set(&u_value, &u_index);
          vss_node.set(&vss_value, &vss_index);

          auto data                   = makeOrderData<order>();
          data.parameters[Params::T2] = T[0];
          data.parameters[Params::T4] = T[1];
          data.parameters[Params::T6] = T[2];

          PhasorDynamics::Stabilizer::Ieeest<ScalarT, IdxT, order> model(data);
          model.getSignals().template attachSignalNode<PhasorDynamics::Stabilizer::IeeestExternalVariables::U>(&u_node);
          model.getSignals().template assignSignalNode<InternalVariables<order>::VSS>(&vss_node);

          model.allocate();
          success *= (model.verify() == 0);
          success *= (model.initialize() == 0);
          success *= (model.evaluateResidual() == 0);

          // The smooth clamp keeps the VSS row only approximately zero.
          const auto  loose_tol = static_cast<RealT>(1.0e-4);
          const auto& residual  = model.getResidual();
          const auto* f         = residual.getData();
          for (size_t i = 0; i < residual.getSize(); ++i)
          {
            if (!isEqual(f[i], static_cast<ScalarT>(0.0), loose_tol))
            {
              std::cout << "Nonzero initial residual at row " << i << ": "
                        << std::setprecision(15) << f[i] << "\n";
              success = false;
            }
          }
        }

        const std::string name = orderedName(__func__, order);
        return success.report(name.c_str());
      }

      template <size_t order>
      TestOutcome residual()
      {
        TestStatus success = true;

        PhasorDynamics::SignalNode<ScalarT, IdxT> u_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> vss_node;
        ScalarT                                   u_value{0.5};
        IdxT                                      u_index{12};
        ScalarT                                   vss_value{0.0};
        IdxT                                      vss_index{INVALID_INDEX<IdxT>};

        u_node.set(&u_value, &u_index);
        vss_node.set(&vss_value, &vss_index);

        PhasorDynamics::Stabilizer::Ieeest<ScalarT, IdxT, order> model(makeOrderData<order>());
        model.getSignals().template attachSignalNode<PhasorDynamics::Stabilizer::IeeestExternalVariables::U>(&u_node);
        model.getSignals().template assignSignalNode<InternalVariables<order>::VSS>(&vss_node);

        model.allocate();
        model.initialize();

        const auto y_values  = stateValues<order>();
        const auto yp_values = derivativeValues<order>();
        auto*      y         = model.y().getData();
        auto*      yp        = model.yp().getData();
        for (size_t i = 0; i < y_values.size(); ++i)
        {
          y[i]  = y_values[i];
          yp[i] = yp_values[i];
        }
        model.y().setDataUpdated();
        model.yp().setDataUpdated();

        model.evaluateResidual();
        const auto* f = model.getResidual().getData();

        // The smooth clamp on the VSS row carries approximation error, so
        // that row is compared with a looser tolerance.
        const auto VSS       = static_cast<size_t>(InternalVariables<order>::VSS);
        const auto loose_tol = static_cast<RealT>(1.0e-4);
        const auto expected  = expectedResidual<order>();
        for (size_t i = 0; i < expected.size(); ++i)
        {
          const auto test_tol = (i == VSS) ? loose_tol : tol_;
          if (!isEqual(f[i], expected[i], test_tol))
          {
            std::cout << "Incorrect residual for order " << order
                      << " row " << i << ": "
                      << std::setprecision(15) << f[i]
                      << " != " << expected[i] << "\n";
            success = false;
          }
        }

        const std::string name = orderedName(__func__, order);
        return success.report(name.c_str());
      }

      template <size_t order>
      TestOutcome tags()
      {
        TestStatus success = true;

        PhasorDynamics::SignalNode<ScalarT, IdxT> u_node;
        ScalarT                                   u_value{0.0};
        IdxT                                      u_index{12};
        u_node.set(&u_value, &u_index);

        PhasorDynamics::Stabilizer::Ieeest<ScalarT, IdxT, order> model(makeOrderData<order>());
        model.getSignals().template attachSignalNode<PhasorDynamics::Stabilizer::IeeestExternalVariables::U>(&u_node);

        model.allocate();

        for (size_t i = 0; i < model.tag().size(); ++i)
        {
          if (model.tag()[i])
          {
            std::cout << "Differential tag set before tagDifferentiable at row " << i << "\n";
            success = false;
          }
        }

        model.tagDifferentiable();

        const auto X7 = static_cast<size_t>(InternalVariables<order>::X7);
        for (size_t i = 0; i < model.tag().size(); ++i)
        {
          const bool expected = (i <= X7);
          if (model.tag()[i] != expected)
          {
            std::cout << "Incorrect differential tag at row " << i << "\n";
            success = false;
          }
        }

        const std::string name = orderedName(__func__, order);
        return success.report(name.c_str());
      }

      TestOutcome verify()
      {
        TestStatus success = true;
        using Params       = PhasorDynamics::Stabilizer::IeeestParameters;

        // Missing input signal fails verification.
        {
          PhasorDynamics::Stabilizer::Ieeest<ScalarT, IdxT, 4> model(makeOrderData<4>());
          model.allocate();
          success *= (model.verify() != 0);
        }

        // Attached but unlinked input signal fails verification.
        {
          PhasorDynamics::SignalNode<ScalarT, IdxT>            u_node;
          PhasorDynamics::Stabilizer::Ieeest<ScalarT, IdxT, 4> model(makeOrderData<4>());
          model.getSignals().template attachSignalNode<PhasorDynamics::Stabilizer::IeeestExternalVariables::U>(&u_node);
          model.allocate();
          success *= (model.verify() != 0);
        }

        // Parameters implying a different order fail verification.
        {
          PhasorDynamics::SignalNode<ScalarT, IdxT> u_node;
          ScalarT                                   u_value{0.0};
          IdxT                                      u_index{12};
          u_node.set(&u_value, &u_index);

          PhasorDynamics::Stabilizer::Ieeest<ScalarT, IdxT, 2> model(makeOrderData<4>());
          model.getSignals().template attachSignalNode<PhasorDynamics::Stabilizer::IeeestExternalVariables::U>(&u_node);
          model.allocate();
          success *= (model.verify() != 0);
        }

        // First-order notch with a second-order numerator fails verification.
        {
          PhasorDynamics::SignalNode<ScalarT, IdxT> u_node;
          ScalarT                                   u_value{0.0};
          IdxT                                      u_index{12};
          u_node.set(&u_value, &u_index);

          auto data                   = makeOrderData<1>();
          data.parameters[Params::A6] = static_cast<RealT>(0.6);

          PhasorDynamics::Stabilizer::Ieeest<ScalarT, IdxT, 1> model(data);
          model.getSignals().template attachSignalNode<PhasorDynamics::Stabilizer::IeeestExternalVariables::U>(&u_node);
          model.allocate();
          success *= (model.verify() != 0);
        }

        // First-order notch with a first-order numerator verifies.
        {
          PhasorDynamics::SignalNode<ScalarT, IdxT> u_node;
          ScalarT                                   u_value{0.0};
          IdxT                                      u_index{12};
          u_node.set(&u_value, &u_index);

          PhasorDynamics::Stabilizer::Ieeest<ScalarT, IdxT, 1> model(makeOrderData<1>());
          model.getSignals().template attachSignalNode<PhasorDynamics::Stabilizer::IeeestExternalVariables::U>(&u_node);
          model.allocate();
          success *= (model.verify() == 0);
        }

        // Zeroth-order notch with a nonzero numerator fails verification.
        {
          PhasorDynamics::SignalNode<ScalarT, IdxT> u_node;
          ScalarT                                   u_value{0.0};
          IdxT                                      u_index{12};
          u_node.set(&u_value, &u_index);

          auto data                   = makeOrderData<0>();
          data.parameters[Params::A5] = static_cast<RealT>(0.5);

          PhasorDynamics::Stabilizer::Ieeest<ScalarT, IdxT, 0> model(data);
          model.getSignals().template attachSignalNode<PhasorDynamics::Stabilizer::IeeestExternalVariables::U>(&u_node);
          model.allocate();
          success *= (model.verify() != 0);
        }

        // Zeroth-order pass-through verifies.
        {
          PhasorDynamics::SignalNode<ScalarT, IdxT> u_node;
          ScalarT                                   u_value{0.0};
          IdxT                                      u_index{12};
          u_node.set(&u_value, &u_index);

          PhasorDynamics::Stabilizer::Ieeest<ScalarT, IdxT, 0> model(makeOrderData<0>());
          model.getSignals().template attachSignalNode<PhasorDynamics::Stabilizer::IeeestExternalVariables::U>(&u_node);
          model.allocate();
          success *= (model.verify() == 0);
        }

        return success.report(__func__);
      }

      TestOutcome factory()
      {
        TestStatus success = true;

        success *= checkFactoryOrder<0>();
        success *= checkFactoryOrder<1>();
        success *= checkFactoryOrder<2>();
        success *= checkFactoryOrder<3>();
        success *= checkFactoryOrder<4>();

        // A null output node is allowed.
        {
          PhasorDynamics::SignalNode<ScalarT, IdxT> u_node;
          ScalarT                                   u_value{0.5};
          IdxT                                      u_index{12};
          u_node.set(&u_value, &u_index);

          auto* stabilizer =
              PhasorDynamics::Stabilizer::StabilizerFactory<ScalarT, IdxT>::create(makeOrderData<4>(), &u_node, nullptr);
          stabilizer->allocate();
          success *= (stabilizer->verify() == 0);
          delete stabilizer;
        }

        // A null input node constructs, but fails verification.
        {
          PhasorDynamics::SignalNode<ScalarT, IdxT> vss_node;
          ScalarT                                   vss_value{0.0};
          IdxT                                      vss_index{INVALID_INDEX<IdxT>};
          vss_node.set(&vss_value, &vss_index);

          auto* stabilizer =
              PhasorDynamics::Stabilizer::StabilizerFactory<ScalarT, IdxT>::create(makeOrderData<4>(), nullptr, &vss_node);
          stabilizer->allocate();
          success *= (stabilizer->verify() != 0);
          delete stabilizer;
        }

        return success.report(__func__);
      }

      /// Symmetric notch filter (A1 = A3 = 0): a standard fourth-order
      /// configuration with zero interior denominator coefficients
      /// (a1 = a3 = 0)
      ///
      /// The Enzyme-vs-dependency-tracking Jacobian comparison is not run for
      /// this configuration: Enzyme's `sparse_store` drops exact-zero entries
      /// (the a1/a3 columns) while dependency tracking retains them, so the
      /// key sets legitimately differ.
      TestOutcome symmetricNotch()
      {
        TestStatus success = true;

        PhasorDynamics::SignalNode<ScalarT, IdxT> u_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> vss_node;
        ScalarT                                   u_value{0.5};
        IdxT                                      u_index{12};
        ScalarT                                   vss_value{0.0};
        IdxT                                      vss_index{INVALID_INDEX<IdxT>};

        u_node.set(&u_value, &u_index);
        vss_node.set(&vss_value, &vss_index);

        auto* stabilizer =
            PhasorDynamics::Stabilizer::StabilizerFactory<ScalarT, IdxT>::create(makeSymmetricNotchData(), &u_node, &vss_node);

        // a = (0, A2 + A4, 0, A2 * A4): the factory must dispatch to order 4.
        success *= (stabilizer->size() == static_cast<IdxT>(InternalVariables<4>::MAXIMUM));
        success *= (stabilizer->allocate() == 0);
        success *= (stabilizer->verify() == 0);
        success *= (stabilizer->initialize() == 0);
        success *= (stabilizer->evaluateResidual() == 0);

        const auto y_values  = stateValues<4>();
        const auto yp_values = derivativeValues<4>();
        auto*      y         = stabilizer->y().getData();
        auto*      yp        = stabilizer->yp().getData();
        for (size_t i = 0; i < y_values.size(); ++i)
        {
          y[i]  = y_values[i];
          yp[i] = yp_values[i];
        }
        stabilizer->y().setDataUpdated();
        stabilizer->yp().setDataUpdated();

        stabilizer->evaluateResidual();
        const auto* f = stabilizer->getResidual().getData();

        // Derived with the a1 and a3 residual terms dropped:
        // x4_rhs = (0.5 - 0.1 - 0.6 * 0.3) / 0.08 = 2.75
        const std::vector<ScalarT> expected =
            {0.19, 0.28, 0.37, 2.71, 0.25, 0.24, -0.01, -0.42, -0.25, -0.31, 1.15, 0.0};

        // The smooth clamp on the VSS row carries approximation error, so
        // that row is compared with a looser tolerance.
        const auto VSS       = static_cast<size_t>(InternalVariables<4>::VSS);
        const auto loose_tol = static_cast<RealT>(1.0e-4);
        for (size_t i = 0; i < expected.size(); ++i)
        {
          const auto test_tol = (i == VSS) ? loose_tol : tol_;
          if (!isEqual(f[i], expected[i], test_tol))
          {
            std::cout << "Incorrect symmetric-notch residual row " << i << ": "
                      << std::setprecision(15) << f[i]
                      << " != " << expected[i] << "\n";
            success = false;
          }
        }

        delete stabilizer;
        return success.report(__func__);
      }

#ifdef GRIDKIT_ENABLE_ENZYME
      template <size_t order>
      TestOutcome jacobian()
      {
        TestStatus success = true;
        using DepVar       = DependencyTracking::Variable;

        // The input signal takes the first global index after the model block
        // so the dependency-tracking and Enzyme variable numbers align.
        const IdxT u_index_value = static_cast<IdxT>(InternalVariables<order>::MAXIMUM);

        const auto y_values  = stateValues<order>();
        const auto yp_values = derivativeValues<order>();

        std::vector<DependencyTracking::Variable::DependencyMap> dependency_tracking_jacobian;

        {
          PhasorDynamics::SignalNode<DepVar, IdxT> u_node;
          PhasorDynamics::SignalNode<DepVar, IdxT> vss_node;
          DepVar                                   u_value{0.5};
          IdxT                                     u_index{u_index_value};
          DepVar                                   vss_value{0.0};
          IdxT                                     vss_index{INVALID_INDEX<IdxT>};

          u_node.set(&u_value, &u_index);
          vss_node.set(&vss_value, &vss_index);

          PhasorDynamics::Stabilizer::Ieeest<DepVar, IdxT, order> model(makeOrderData<order>());
          model.getSignals().template attachSignalNode<PhasorDynamics::Stabilizer::IeeestExternalVariables::U>(&u_node);
          model.getSignals().template assignSignalNode<InternalVariables<order>::VSS>(&vss_node);

          model.allocate();
          model.initialize();

          auto* y  = model.y().getData();
          auto* yp = model.yp().getData();
          for (size_t i = 0; i < model.size(); ++i)
          {
            y[i].setVariableNumber(i);
          }
          u_value.setVariableNumber(model.size());
          u_value.setValue(0.5);

          for (size_t i = 0; i < y_values.size(); ++i)
          {
            y[i].setValue(y_values[i]);
          }
          for (size_t i = 0; i < yp_values.size(); ++i)
          {
            yp[i].setValue(yp_values[i]);
          }
          model.y().setDataUpdated();
          model.yp().setDataUpdated();

          model.evaluateResidual();
          const auto&         residual_y_view = model.getResidual();
          std::vector<DepVar> residual_y(residual_y_view.getData(),
                                         residual_y_view.getData() + residual_y_view.getSize());

          model.initialize();
          for (size_t i = 0; i < model.size(); ++i)
          {
            y[i] = y[i].getValue();
            yp[i].setVariableNumber(i);
          }
          u_value = 0.5;

          for (size_t i = 0; i < y_values.size(); ++i)
          {
            y[i].setValue(y_values[i]);
          }
          for (size_t i = 0; i < yp_values.size(); ++i)
          {
            yp[i].setValue(yp_values[i]);
          }
          model.y().setDataUpdated();
          model.yp().setDataUpdated();

          model.evaluateResidual();
          const auto&         residual_yp_view = model.getResidual();
          std::vector<DepVar> residual_yp(residual_yp_view.getData(),
                                          residual_yp_view.getData() + residual_yp_view.getSize());

          dependency_tracking_jacobian.resize(residual_y.size());
          for (size_t i = 0; i < residual_y.size(); ++i)
          {
            auto dependency_y  = residual_y[i].getDependencies();
            auto dependency_yp = residual_yp[i].getDependencies();

            for (const auto& pair_y : dependency_y)
            {
              auto it_yp = dependency_yp.find(pair_y.first);
              if (it_yp != dependency_yp.end())
              {
                dependency_tracking_jacobian[i].insert(std::make_pair(pair_y.first, pair_y.second + it_yp->second));
              }
              else
              {
                dependency_tracking_jacobian[i].insert(std::make_pair(pair_y.first, pair_y.second));
              }
            }

            for (const auto& pair_yp : dependency_yp)
            {
              if (!dependency_y.contains(pair_yp.first))
              {
                dependency_tracking_jacobian[i].insert(std::make_pair(pair_yp.first, pair_yp.second));
              }
            }
          }
        }

        std::vector<DependencyTracking::Variable::DependencyMap> enzyme_jacobian;

        {
          PhasorDynamics::SignalNode<ScalarT, IdxT> u_node;
          PhasorDynamics::SignalNode<ScalarT, IdxT> vss_node;
          ScalarT                                   u_value{0.5};
          IdxT                                      u_index{u_index_value};
          ScalarT                                   vss_value{0.0};
          IdxT                                      vss_index{INVALID_INDEX<IdxT>};

          u_node.set(&u_value, &u_index);
          vss_node.set(&vss_value, &vss_index);

          PhasorDynamics::Stabilizer::Ieeest<ScalarT, IdxT, order> model(makeOrderData<order>());
          model.getSignals().template attachSignalNode<PhasorDynamics::Stabilizer::IeeestExternalVariables::U>(&u_node);
          model.getSignals().template assignSignalNode<InternalVariables<order>::VSS>(&vss_node);

          model.allocate();
          model.initialize();

          auto* y  = model.y().getData();
          auto* yp = model.yp().getData();
          for (size_t i = 0; i < y_values.size(); ++i)
          {
            y[i]  = y_values[i];
            yp[i] = yp_values[i];
          }
          model.y().setDataUpdated();
          model.yp().setDataUpdated();

          model.updateTime(0.0, 1.0);
          model.evaluateResidual();
          model.evaluateJacobian();
          model.constructCsr();
          auto model_jacobian = model.getCsrJacobian();
          enzyme_jacobian     = GridKit::Testing::MapFromCsr(model_jacobian);
        }

        for (size_t i = 0; i < dependency_tracking_jacobian.size(); ++i)
        {
          success *= GridKit::Testing::isEqual(dependency_tracking_jacobian[i], enzyme_jacobian[i], tol_);
        }

        const std::string name = orderedName(__func__, order);
        return success.report(name.c_str());
      }
#endif

    private:
      static constexpr ScalarT tol_ = 10 * std::numeric_limits<ScalarT>::epsilon();

      static std::string orderedName(const char* funcname, size_t order)
      {
        return std::string(funcname) + " (order " + std::to_string(order) + ")";
      }

      template <size_t order>
      bool checkConstructedSize()
      {
        PhasorDynamics::Stabilizer::Ieeest<ScalarT, IdxT, order> model(makeOrderData<order>());

        bool success = (model.size() == static_cast<IdxT>(InternalVariables<order>::MAXIMUM));
        // `order` notch states + X5..X7 + V4..V7 + VSS
        success      = success && (model.size() == static_cast<IdxT>(order + 8));
        success      = success && (model.getMonitor() != nullptr);

        return success;
      }

      template <size_t order>
      bool checkFactoryOrder()
      {
        PhasorDynamics::SignalNode<ScalarT, IdxT> u_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> vss_node;
        ScalarT                                   u_value{0.25};
        IdxT                                      u_index{12};
        ScalarT                                   vss_value{0.0};
        IdxT                                      vss_index{INVALID_INDEX<IdxT>};

        u_node.set(&u_value, &u_index);
        vss_node.set(&vss_value, &vss_index);

        auto* stabilizer =
            PhasorDynamics::Stabilizer::StabilizerFactory<ScalarT, IdxT>::create(makeOrderData<order>(), &u_node, &vss_node);

        bool success = (stabilizer->size() == static_cast<IdxT>(InternalVariables<order>::MAXIMUM));
        success      = success && (stabilizer->allocate() == 0);
        success      = success && (stabilizer->verify() == 0);
        success      = success && (stabilizer->initialize() == 0);
        success      = success && vss_node.linked();
        success      = success && (vss_node.getVariableIndex() == static_cast<IdxT>(InternalVariables<order>::VSS));

        if (!success)
        {
          std::cout << "Factory checks failed for order " << order << "\n";
        }

        delete stabilizer;
        return success;
      }

      auto makeData() -> DataT
      {
        using Params = PhasorDynamics::Stabilizer::IeeestParameters;

        DataT data;
        data.device_class          = "stabilizer";
        data.disambiguation_string = "ieeest_test";
        data.monitored_variables.insert(PhasorDynamics::Stabilizer::IeeestMonitorableVariables::vss);

        data.parameters[Params::A1]     = 0.1;
        data.parameters[Params::A2]     = 0.2;
        data.parameters[Params::A3]     = 0.3;
        data.parameters[Params::A4]     = 0.4;
        data.parameters[Params::A5]     = 0.5;
        data.parameters[Params::A6]     = 0.6;
        data.parameters[Params::T1]     = 0.5;
        data.parameters[Params::T2]     = 1.0;
        data.parameters[Params::T3]     = 0.3;
        data.parameters[Params::T4]     = 1.0;
        data.parameters[Params::T5]     = 2.0;
        data.parameters[Params::T6]     = 5.0;
        data.parameters[Params::Ks]     = 10.0;
        data.parameters[Params::Lsmin]  = -0.1;
        data.parameters[Params::Lsmax]  = 0.1;
        data.parameters[Params::Vcl]    = 0.0;
        data.parameters[Params::Vcu]    = 0.0;
        data.parameters[Params::Tdelay] = 0.0;

        return data;
      }

      /// Base data with A1 = A3 = 0: still fourth order, with a1 = a3 = 0
      auto makeSymmetricNotchData() -> DataT
      {
        using Params = PhasorDynamics::Stabilizer::IeeestParameters;

        auto data                   = makeData();
        data.parameters[Params::A1] = static_cast<RealT>(0.0);
        data.parameters[Params::A3] = static_cast<RealT>(0.0);

        return data;
      }

      /// Base data edited so the derived notch-filter order equals `order`
      template <size_t order>
      auto makeOrderData() -> DataT
      {
        using Params = PhasorDynamics::Stabilizer::IeeestParameters;

        auto data = makeData();

        if constexpr (order == 3)
        {
          data.parameters[Params::A4] = static_cast<RealT>(0.0);
        }
        else if constexpr (order == 2)
        {
          data.parameters[Params::A3] = static_cast<RealT>(0.0);
          data.parameters[Params::A4] = static_cast<RealT>(0.0);
        }
        else if constexpr (order == 1)
        {
          data.parameters[Params::A2] = static_cast<RealT>(0.0);
          data.parameters[Params::A3] = static_cast<RealT>(0.0);
          data.parameters[Params::A4] = static_cast<RealT>(0.0);
          data.parameters[Params::A6] = static_cast<RealT>(0.0);
        }
        else if constexpr (order == 0)
        {
          data.parameters[Params::A1] = static_cast<RealT>(0.0);
          data.parameters[Params::A2] = static_cast<RealT>(0.0);
          data.parameters[Params::A3] = static_cast<RealT>(0.0);
          data.parameters[Params::A4] = static_cast<RealT>(0.0);
          data.parameters[Params::A5] = static_cast<RealT>(0.0);
          data.parameters[Params::A6] = static_cast<RealT>(0.0);
        }

        return data;
      }

      /// Test state: every variable keeps its named value at every order
      /// (x1..x4 = 0.1..0.4, x5..x7 = 0.5..0.7, v4..v6 = 0.8..1.0,
      /// v7 = vss = 0.05), so shared residual rows match across orders.
      template <size_t order>
      static std::vector<RealT> stateValues()
      {
        if constexpr (order == 0)
        {
          return {0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 0.05, 0.05};
        }
        else if constexpr (order == 1)
        {
          return {0.1, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 0.05, 0.05};
        }
        else if constexpr (order == 2)
        {
          return {0.1, 0.2, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 0.05, 0.05};
        }
        else if constexpr (order == 3)
        {
          return {0.1, 0.2, 0.3, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 0.05, 0.05};
        }
        else
        {
          return {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 0.05, 0.05};
        }
      }

      /// Test state derivatives (differential block only; algebraic rows zero)
      template <size_t order>
      static std::vector<RealT> derivativeValues()
      {
        if constexpr (order == 0)
        {
          return {0.05, 0.06, 0.07, 0.0, 0.0, 0.0, 0.0, 0.0};
        }
        else if constexpr (order == 1)
        {
          return {0.01, 0.05, 0.06, 0.07, 0.0, 0.0, 0.0, 0.0, 0.0};
        }
        else if constexpr (order == 2)
        {
          return {0.01, 0.02, 0.05, 0.06, 0.07, 0.0, 0.0, 0.0, 0.0, 0.0};
        }
        else if constexpr (order == 3)
        {
          return {0.01, 0.02, 0.03, 0.05, 0.06, 0.07, 0.0, 0.0, 0.0, 0.0, 0.0};
        }
        else
        {
          return {0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.0, 0.0, 0.0, 0.0, 0.0};
        }
      }

      /// Expected residuals for `stateValues`/`derivativeValues` with
      /// `makeOrderData` and u = 0.5, derived from the model equations
      template <size_t order>
      static std::vector<ScalarT> expectedResidual()
      {
        if constexpr (order == 0)
        {
          return {0.25, 0.24, -0.01, -0.3, -0.25, -0.31, 1.15, 0.0};
        }
        else if constexpr (order == 1)
        {
          return {3.99, 0.25, 0.24, -0.01, 1.3, -0.25, -0.31, 1.15, 0.0};
        }
        else if constexpr (order == 2)
        {
          return {0.19, 1.88, 0.25, 0.24, -0.01, 0.54, -0.25, -0.31, 1.15, 0.0};
        }
        else if constexpr (order == 3)
        {
          return {0.19, 0.28, 4.153333333333333, 0.25, 0.24, -0.01, -0.42, -0.25, -0.31, 1.15, 0.0};
        }
        else
        {
          return {0.19, 0.28, 0.37, 1.0975, 0.25, 0.24, -0.01, -0.42, -0.25, -0.31, 1.15, 0.0};
        }
      }
    }; // class StabilizerIeeestTests

  } // namespace Testing
} // namespace GridKit

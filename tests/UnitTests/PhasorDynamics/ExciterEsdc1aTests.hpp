#pragma once

#include <cmath>
#include <iostream>
#include <vector>

#include <GridKit/AutomaticDifferentiation/DependencyTracking/Variable.hpp>
#include <GridKit/Definitions.hpp>
#include <GridKit/Model/PhasorDynamics/Bus/Bus.hpp>
#include <GridKit/Model/PhasorDynamics/Exciter/ESDC1A/Esdc1a.hpp>
#include <GridKit/Model/PhasorDynamics/Exciter/ESDC1A/Esdc1aData.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Testing/TestHelpers.hpp>
#include <GridKit/Testing/Testing.hpp>
#include <GridKit/Utilities/MapFromCsr.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class ExciterEsdc1aTests
    {
    public:
      using RealT = typename PhasorDynamics::Component<ScalarT, IdxT>::RealT;

      ExciterEsdc1aTests()  = default;
      ~ExciterEsdc1aTests() = default;

      static constexpr ScalarT kTol = static_cast<ScalarT>(1.0e-12);

      TestOutcome constructor()
      {
        using namespace PhasorDynamics::Exciter;

        TestStatus success = true;

        PhasorDynamics::Bus<ScalarT, IdxT> bus(3.0, 4.0);

        Esdc1a<ScalarT, IdxT> default_exciter(&bus);
        success *= (default_exciter.size() == static_cast<IdxT>(Esdc1aInternalVariables::MAXIMUM));
        success *= (default_exciter.getMonitor() == nullptr);

        auto data = makeDefaultData();
        data.monitored_variables.insert(Esdc1aMonitorableVariables::efd);
        Esdc1a<ScalarT, IdxT> data_exciter(&bus, data);
        success *= (data_exciter.size() == static_cast<IdxT>(Esdc1aInternalVariables::MAXIMUM));
        success *= (data_exciter.getMonitor() != nullptr);

        PhasorDynamics::SignalNode<ScalarT, IdxT> efd_node;
        ScalarT                                   efd_value{0.0};
        IdxT                                      efd_index = INVALID_INDEX<IdxT>;
        efd_node.set(&efd_value, &efd_index);

        data_exciter.getSignals().template assignSignalNode<Esdc1aInternalVariables::EFD>(&efd_node);
        data_exciter.allocate();
        data_exciter.tagDifferentiable();

        success *= (data_exciter.verify() == 0);
        success *= (data_exciter.tag()[idx(Esdc1aInternalVariables::EFDP)] == true);
        success *= (data_exciter.tag()[idx(Esdc1aInternalVariables::VC)] == true);
        success *= (data_exciter.tag()[idx(Esdc1aInternalVariables::VR)] == true);
        success *= (data_exciter.tag()[idx(Esdc1aInternalVariables::VF)] == true);
        success *= (data_exciter.tag()[idx(Esdc1aInternalVariables::XLL)] == true);

        return success.report(__func__);
      }

      TestOutcome zeroInitialResidual()
      {
        using namespace PhasorDynamics::Exciter;

        TestStatus success = true;

        Fixture fixture(makeDefaultData());
        success *= fixture.allocateAndInitialize(1.2);

        const auto& f = fixture.exciter.getResidual();
        for (size_t i = 0; i < f.size(); ++i)
        {
          if (!isEqual(f[i], static_cast<ScalarT>(0.0), kTol))
          {
            std::cout << "Non-zero ESDC1A residual at index " << i << ": " << f[i] << "\n";
            success = false;
          }
        }

        success *= fixture.efd_node.linked();
        success *= (fixture.efd_node.getVariableIndex()
                    == static_cast<IdxT>(idx(Esdc1aInternalVariables::EFD)));
        success *= isEqual(fixture.efd_node.read(), static_cast<ScalarT>(1.2), kTol);
        success *= smoothHighValueGateInitialResidual();

        return success.report(__func__);
      }

      TestOutcome blockDiagramSemantics()
      {
        TestStatus success = true;

        success *= voltageErrorSummingJunction();
        success *= speedMultiplierSelector();
        success *= leadLagBlockSemantics();
        success *= timeConstantClampSemantics();
        success *= uelRoutingSelector();
        success *= exciterFeedbackLimiter();

        return success.report(__func__);
      }

      TestOutcome parameterValidation()
      {
        using namespace PhasorDynamics::Exciter;

        TestStatus success = true;

        PhasorDynamics::Bus<ScalarT, IdxT>        bus(1.0, 0.0);
        PhasorDynamics::SignalNode<ScalarT, IdxT> efd_node;
        ScalarT                                   efd_value{0.0};
        IdxT                                      efd_index = INVALID_INDEX<IdxT>;
        efd_node.set(&efd_value, &efd_index);

        auto                  valid = makeDefaultData();
        Esdc1a<ScalarT, IdxT> valid_model(&bus, valid);
        valid_model.getSignals().template assignSignalNode<Esdc1aInternalVariables::EFD>(&efd_node);
        valid_model.allocate();
        success *= (valid_model.verify() == 0);

        auto invalid_ta                             = makeDefaultData();
        invalid_ta.parameters[Esdc1aParameters::Ta] = 0.0;
        Esdc1a<ScalarT, IdxT> invalid_ta_model(&bus, invalid_ta);
        invalid_ta_model.getSignals().template assignSignalNode<Esdc1aInternalVariables::EFD>(&efd_node);
        invalid_ta_model.allocate();
        success *= (invalid_ta_model.verify() > 0);

        auto zero_tb_nonzero_tc                             = makeDefaultData();
        zero_tb_nonzero_tc.parameters[Esdc1aParameters::Tc] = 0.1;
        Esdc1a<ScalarT, IdxT> zero_tb_nonzero_tc_model(&bus, zero_tb_nonzero_tc);
        zero_tb_nonzero_tc_model.getSignals().template assignSignalNode<Esdc1aInternalVariables::EFD>(&efd_node);
        zero_tb_nonzero_tc_model.allocate();
        success *= (zero_tb_nonzero_tc_model.verify() == 0);

        auto invalid_tc                             = makeDefaultData();
        invalid_tc.parameters[Esdc1aParameters::Tc] = -0.1;
        Esdc1a<ScalarT, IdxT> invalid_tc_model(&bus, invalid_tc);
        invalid_tc_model.getSignals().template assignSignalNode<Esdc1aInternalVariables::EFD>(&efd_node);
        invalid_tc_model.allocate();
        success *= (invalid_tc_model.verify() > 0);

        auto invalid_saturation                              = makeDefaultData();
        invalid_saturation.parameters[Esdc1aParameters::Se1] = 0.0;
        invalid_saturation.parameters[Esdc1aParameters::Se2] = 0.33;
        Esdc1a<ScalarT, IdxT> invalid_saturation_model(&bus, invalid_saturation);
        invalid_saturation_model.getSignals().template assignSignalNode<Esdc1aInternalVariables::EFD>(&efd_node);
        invalid_saturation_model.allocate();
        success *= (invalid_saturation_model.verify() > 0);

        auto                  missing_efd = makeDefaultData();
        Esdc1a<ScalarT, IdxT> missing_efd_model(&bus, missing_efd);
        missing_efd_model.allocate();
        success *= (missing_efd_model.verify() > 0);

        auto missing_speed                                 = makeDefaultData();
        missing_speed.parameters[Esdc1aParameters::Spdmlt] = 1.0;
        Esdc1a<ScalarT, IdxT> missing_speed_model(&bus, missing_speed);
        missing_speed_model.getSignals().template assignSignalNode<Esdc1aInternalVariables::EFD>(&efd_node);
        missing_speed_model.allocate();
        success *= (missing_speed_model.verify() > 0);

        return success.report(__func__);
      }

#ifdef GRIDKIT_ENABLE_ENZYME
      TestOutcome jacobianStructureAndValues()
      {
        TestStatus success = true;

        const auto tol = static_cast<RealT>(1.0e-9);

        auto data                       = makeDefaultData();
        data.parameters[Params::Spdmlt] = 1.0;
        data.parameters[Params::UEL]    = static_cast<IdxT>(2);

        auto dependency_tracking_jacobian = dependencyTrackingJacobian(data);
        auto enzyme_jacobian              = enzymeJacobian(data);

        success *= (dependency_tracking_jacobian.size() == enzyme_jacobian.size());
        for (size_t i = 0; i < dependency_tracking_jacobian.size(); ++i)
        {
          success *= isEqual(dependency_tracking_jacobian[i], enzyme_jacobian[i], tol);
        }

        return success.report(__func__);
      }
#endif

    private:
      using Internal = PhasorDynamics::Exciter::Esdc1aInternalVariables;
      using External = PhasorDynamics::Exciter::Esdc1aExternalVariables;
      using Params   = PhasorDynamics::Exciter::Esdc1aParameters;
      using DataT    = PhasorDynamics::Exciter::Esdc1aData<RealT, IdxT>;

      static size_t idx(Internal variable)
      {
        return static_cast<size_t>(variable);
      }

      auto makeDefaultData() -> DataT
      {
        DataT data;
        data.device_class          = "exciter";
        data.disambiguation_string = "esdc1a_test";

        data.parameters[Params::Tr]     = 0.0;
        data.parameters[Params::Ka]     = 40.0;
        data.parameters[Params::Ta]     = 0.1;
        data.parameters[Params::Tb]     = 0.0;
        data.parameters[Params::Tc]     = 0.0;
        data.parameters[Params::Vrmax]  = 1.0;
        data.parameters[Params::Vrmin]  = -1.0;
        data.parameters[Params::Ke]     = 0.1;
        data.parameters[Params::Te]     = 0.5;
        data.parameters[Params::Kf]     = 0.05;
        data.parameters[Params::Tf1]    = 0.7;
        data.parameters[Params::Spdmlt] = 0.0;
        data.parameters[Params::E1]     = 2.8;
        data.parameters[Params::Se1]    = 0.08;
        data.parameters[Params::E2]     = 3.7;
        data.parameters[Params::Se2]    = 0.33;
        data.parameters[Params::UEL]    = static_cast<IdxT>(0);
        data.parameters[Params::exclim] = 1.0;

        return data;
      }

      struct Fixture
      {
        using BusT     = PhasorDynamics::Bus<ScalarT, IdxT>;
        using SignalT  = PhasorDynamics::SignalNode<ScalarT, IdxT>;
        using ExciterT = PhasorDynamics::Exciter::Esdc1a<ScalarT, IdxT>;

        DataT   data;
        BusT    bus;
        SignalT efd_node;
        SignalT omega_node;
        SignalT vs_node;
        SignalT vuel_node;

        ScalarT efd_value{0.0};
        ScalarT omega_value{0.0};
        ScalarT vs_value{0.0};
        ScalarT vuel_value{-2.0};

        IdxT efd_index{INVALID_INDEX<IdxT>};
        IdxT omega_index{20};
        IdxT vs_index{21};
        IdxT vuel_index{22};

        ExciterT exciter;

        explicit Fixture(const DataT& data_in)
          : data(data_in),
            bus(3.0, 4.0),
            exciter(&bus, data)
        {
          efd_node.set(&efd_value, &efd_index);
          omega_node.set(&omega_value, &omega_index);
          vs_node.set(&vs_value, &vs_index);
          vuel_node.set(&vuel_value, &vuel_index);

          exciter.getSignals().template assignSignalNode<Internal::EFD>(&efd_node);
          exciter.getSignals().template attachSignalNode<External::OMEGA>(&omega_node);
          exciter.getSignals().template attachSignalNode<External::VS>(&vs_node);
          exciter.getSignals().template attachSignalNode<External::VUEL>(&vuel_node);
        }

        bool allocateAndInitialize(ScalarT efd0)
        {
          bus.allocate();
          bus.initialize();
          exciter.allocate();
          efd_node.init(efd0);
          return exciter.verify() == 0
                 && exciter.initialize() == 0
                 && exciter.evaluateResidual() == 0;
        }
      };

      bool voltageErrorSummingJunction()
      {
        Fixture fixture(makeDefaultData());
        if (!fixture.allocateAndInitialize(1.2))
        {
          return false;
        }

        fixture.vs_value += 0.1;
        fixture.exciter.evaluateResidual();
        bool success = fixture.exciter.getResidual()[idx(Internal::EV)] > static_cast<ScalarT>(0.0);

        fixture.vs_value                       -= 0.1;
        fixture.exciter.y()[idx(Internal::VC)] += 0.1;
        fixture.exciter.evaluateResidual();
        success = success && fixture.exciter.getResidual()[idx(Internal::EV)] < static_cast<ScalarT>(0.0);

        fixture.exciter.y()[idx(Internal::VC)] -= 0.1;
        fixture.exciter.y()[idx(Internal::VF)] += 0.1;
        fixture.exciter.evaluateResidual();
        success = success && fixture.exciter.getResidual()[idx(Internal::EV)] < static_cast<ScalarT>(0.0);

        return success;
      }

      bool speedMultiplierSelector()
      {
        auto    disabled_data = makeDefaultData();
        Fixture disabled(disabled_data);
        if (!disabled.allocateAndInitialize(1.2))
        {
          return false;
        }

        disabled.omega_value = 0.05;
        disabled.exciter.evaluateResidual();
        bool success = isEqual(disabled.exciter.getResidual()[idx(Internal::EFD)],
                               static_cast<ScalarT>(0.0),
                               kTol);

        auto enabled_data                       = makeDefaultData();
        enabled_data.parameters[Params::Spdmlt] = 1.0;
        Fixture enabled(enabled_data);
        if (!enabled.allocateAndInitialize(1.2))
        {
          return false;
        }

        enabled.omega_value = 0.05;
        enabled.exciter.evaluateResidual();
        success = success && enabled.exciter.getResidual()[idx(Internal::EFD)] > static_cast<ScalarT>(0.0);

        return success;
      }

      bool leadLagBlockSemantics()
      {
        Fixture clamped(makeDefaultData());
        if (!clamped.allocateAndInitialize(1.2))
        {
          return false;
        }

        clamped.exciter.y()[idx(Internal::VLL)] += 0.1;
        clamped.exciter.evaluateResidual();
        bool success = clamped.exciter.getResidual()[idx(Internal::VLL)] < static_cast<ScalarT>(0.0);

        auto active_data                   = makeDefaultData();
        active_data.parameters[Params::Tb] = 0.5;
        active_data.parameters[Params::Tc] = 0.2;
        Fixture active(active_data);
        if (!active.allocateAndInitialize(1.2))
        {
          return false;
        }

        active.exciter.y()[idx(Internal::EV)] += 0.1;
        active.exciter.evaluateResidual();
        success = success && active.exciter.getResidual()[idx(Internal::VLL)] > static_cast<ScalarT>(0.0);

        active.exciter.y()[idx(Internal::EV)]  -= 0.1;
        active.exciter.y()[idx(Internal::VLL)] += 0.1;
        active.exciter.evaluateResidual();
        success = success && active.exciter.getResidual()[idx(Internal::VLL)] < static_cast<ScalarT>(0.0);

        return success;
      }

      bool smoothHighValueGateInitialResidual()
      {
        auto data = makeDefaultData();

        PhasorDynamics::Bus<ScalarT, IdxT>        bus(3.0, 4.0);
        PhasorDynamics::SignalNode<ScalarT, IdxT> efd_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> omega_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> vs_node;

        ScalarT efd_value{0.0};
        ScalarT omega_value{0.0};
        ScalarT vs_value{0.0};

        IdxT efd_index{INVALID_INDEX<IdxT>};
        IdxT omega_index{20};
        IdxT vs_index{21};

        efd_node.set(&efd_value, &efd_index);
        omega_node.set(&omega_value, &omega_index);
        vs_node.set(&vs_value, &vs_index);

        PhasorDynamics::Exciter::Esdc1a<ScalarT, IdxT> exciter(&bus, data);
        exciter.getSignals().template assignSignalNode<Internal::EFD>(&efd_node);
        exciter.getSignals().template attachSignalNode<External::OMEGA>(&omega_node);
        exciter.getSignals().template attachSignalNode<External::VS>(&vs_node);

        bus.allocate();
        bus.initialize();
        exciter.allocate();
        efd_node.init(1.2);

        TestStatus success  = true;
        success            *= (exciter.verify() == 0);
        success            *= (exciter.initialize() == 0);
        success            *= (exciter.evaluateResidual() == 0);
        success            *= isEqual(exciter.getResidual()[idx(Internal::VHV)], static_cast<ScalarT>(0.0), kTol);

        return success;
      }

      bool timeConstantClampSemantics()
      {
        auto data                    = makeDefaultData();
        data.parameters[Params::Tr]  = 0.0;
        data.parameters[Params::Tb]  = 0.0;
        data.parameters[Params::Tc]  = 0.1;
        data.parameters[Params::Tf1] = 0.0;

        Fixture fixture(data);
        if (!fixture.allocateAndInitialize(1.2))
        {
          return false;
        }

        fixture.exciter.tagDifferentiable();

        bool success = fixture.exciter.tag()[idx(Internal::VC)];
        success      = success && fixture.exciter.tag()[idx(Internal::VF)];
        success      = success && fixture.exciter.tag()[idx(Internal::XLL)];

        fixture.exciter.y()[idx(Internal::VC)] += 0.1;
        fixture.exciter.evaluateResidual();
        success = success
                  && fixture.exciter.getResidual()[idx(Internal::VC)]
                         < static_cast<ScalarT>(0.0);

        fixture.exciter.y()[idx(Internal::VC)] -= 0.1;
        fixture.exciter.y()[idx(Internal::VF)] += 0.1;
        fixture.exciter.evaluateResidual();
        success = success
                  && fixture.exciter.getResidual()[idx(Internal::VF)]
                         < static_cast<ScalarT>(0.0);

        fixture.exciter.y()[idx(Internal::VF)]  -= 0.1;
        fixture.exciter.y()[idx(Internal::XLL)] += 0.1;
        fixture.exciter.evaluateResidual();
        success = success
                  && fixture.exciter.getResidual()[idx(Internal::XLL)]
                         < static_cast<ScalarT>(0.0);

        return success;
      }

      bool uelRoutingSelector()
      {
        Fixture hv_gate(makeDefaultData());
        if (!hv_gate.allocateAndInitialize(1.2))
        {
          return false;
        }

        hv_gate.vuel_value = hv_gate.exciter.y()[idx(Internal::VLL)] + 0.1;
        hv_gate.exciter.evaluateResidual();
        bool success = hv_gate.exciter.getResidual()[idx(Internal::VHV)] > static_cast<ScalarT>(0.0);
        success      = success && isEqual(hv_gate.exciter.getResidual()[idx(Internal::EV)], static_cast<ScalarT>(0.0), kTol);

        auto sum_data                    = makeDefaultData();
        sum_data.parameters[Params::UEL] = static_cast<IdxT>(2);
        Fixture sum_junction(sum_data);
        sum_junction.vuel_value = 0.0;
        if (!sum_junction.allocateAndInitialize(1.2))
        {
          return false;
        }

        sum_junction.vuel_value = 0.1;
        sum_junction.exciter.evaluateResidual();
        success = success && sum_junction.exciter.getResidual()[idx(Internal::EV)] > static_cast<ScalarT>(0.0);
        success = success && isEqual(sum_junction.exciter.getResidual()[idx(Internal::VHV)], static_cast<ScalarT>(0.0), kTol);

        return success;
      }

      bool exciterFeedbackLimiter()
      {
        auto limited_data                       = makeDefaultData();
        limited_data.parameters[Params::Ke]     = -0.2;
        limited_data.parameters[Params::Se1]    = 0.0;
        limited_data.parameters[Params::Se2]    = 0.0;
        limited_data.parameters[Params::exclim] = 1.0;
        Fixture limited(limited_data);
        if (!limited.allocateAndInitialize(1.2))
        {
          return false;
        }

        limited.exciter.y()[idx(Internal::EFDP)] = 1.0;
        limited.exciter.y()[idx(Internal::SE)]   = 0.0;
        limited.exciter.y()[idx(Internal::VFE)]  = 0.0;
        limited.exciter.evaluateResidual();
        bool success = std::abs(limited.exciter.getResidual()[idx(Internal::VFE)]) < kTol;

        auto unlimited_data                       = limited_data;
        unlimited_data.parameters[Params::exclim] = 0.0;
        Fixture unlimited(unlimited_data);
        if (!unlimited.allocateAndInitialize(1.2))
        {
          return false;
        }

        unlimited.exciter.y()[idx(Internal::EFDP)] = 1.0;
        unlimited.exciter.y()[idx(Internal::SE)]   = 0.0;
        unlimited.exciter.y()[idx(Internal::VFE)]  = 0.0;
        unlimited.exciter.evaluateResidual();
        success = success && unlimited.exciter.getResidual()[idx(Internal::VFE)] < static_cast<ScalarT>(0.0);

        return success;
      }

#ifdef GRIDKIT_ENABLE_ENZYME
      std::vector<DependencyTracking::Variable::DependencyMap>
      dependencyTrackingJacobian(const DataT& data)
      {
        using Variable = DependencyTracking::Variable;

        PhasorDynamics::Bus<Variable, IdxT>        bus(Variable{3.0}, Variable{4.0});
        PhasorDynamics::SignalNode<Variable, IdxT> efd_node;
        PhasorDynamics::SignalNode<Variable, IdxT> omega_node;
        PhasorDynamics::SignalNode<Variable, IdxT> vs_node;
        PhasorDynamics::SignalNode<Variable, IdxT> vuel_node;

        Variable efd_value{0.0};
        Variable omega_value{0.0};
        Variable vs_value{0.0};
        Variable vuel_value{0.0};

        IdxT efd_index   = INVALID_INDEX<IdxT>;
        IdxT omega_index = 13;
        IdxT vs_index    = 14;
        IdxT vuel_index  = 15;

        efd_node.set(&efd_value, &efd_index);
        omega_node.set(&omega_value, &omega_index);
        vs_node.set(&vs_value, &vs_index);
        vuel_node.set(&vuel_value, &vuel_index);

        PhasorDynamics::Exciter::Esdc1a<Variable, IdxT> exciter(&bus, data);
        exciter.getSignals().template assignSignalNode<Internal::EFD>(&efd_node);
        exciter.getSignals().template attachSignalNode<External::OMEGA>(&omega_node);
        exciter.getSignals().template attachSignalNode<External::VS>(&vs_node);
        exciter.getSignals().template attachSignalNode<External::VUEL>(&vuel_node);

        bus.allocate();
        exciter.allocate();
        bus.initialize();
        efd_node.init(Variable{1.2});
        exciter.initialize();

        for (size_t i = 0; i < exciter.size(); ++i)
        {
          exciter.y()[i].setVariableNumber(i);
        }
        for (size_t i = 0; i < bus.size(); ++i)
        {
          bus.y()[i].setVariableNumber(i + exciter.size());
        }
        omega_value.setVariableNumber(13);
        vs_value.setVariableNumber(14);
        vuel_value.setVariableNumber(15);

        bus.evaluateResidual();
        exciter.evaluateResidual();
        auto residual_y = exciter.getResidual();

        omega_value = 0.0;
        vs_value    = 0.0;
        vuel_value  = 0.0;
        bus.initialize();
        efd_node.init(Variable{1.2});
        exciter.initialize();

        for (size_t i = 0; i < exciter.size(); ++i)
        {
          exciter.yp()[i].setVariableNumber(i);
        }

        bus.evaluateResidual();
        exciter.evaluateResidual();
        auto residual_yp = exciter.getResidual();

        std::vector<Variable::DependencyMap> dependencies(residual_y.size());
        for (IdxT i = 0; i < residual_y.size(); ++i)
        {
          auto dependency_y  = residual_y[static_cast<size_t>(i)].getDependencies();
          auto dependency_yp = residual_yp[static_cast<size_t>(i)].getDependencies();

          for (const auto& pair_y : dependency_y)
          {
            auto index_y = pair_y.first;
            auto value_y = pair_y.second;
            auto it_yp   = dependency_yp.find(index_y);
            if (it_yp != dependency_yp.end())
            {
              dependencies[static_cast<size_t>(i)].insert(std::make_pair(index_y, value_y + it_yp->second));
            }
            else
            {
              dependencies[static_cast<size_t>(i)].insert(pair_y);
            }
          }

          for (const auto& pair_yp : dependency_yp)
          {
            if (dependency_y.find(pair_yp.first) == dependency_y.end())
            {
              dependencies[static_cast<size_t>(i)].insert(pair_yp);
            }
          }
        }

        return dependencies;
      }

      std::vector<DependencyTracking::Variable::DependencyMap>
      enzymeJacobian(const DataT& data)
      {
        PhasorDynamics::Bus<ScalarT, IdxT>        bus(3.0, 4.0);
        PhasorDynamics::SignalNode<ScalarT, IdxT> efd_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> omega_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> vs_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> vuel_node;

        ScalarT efd_value{0.0};
        ScalarT omega_value{0.0};
        ScalarT vs_value{0.0};
        ScalarT vuel_value{0.0};

        IdxT efd_index   = INVALID_INDEX<IdxT>;
        IdxT omega_index = 13;
        IdxT vs_index    = 14;
        IdxT vuel_index  = 15;

        efd_node.set(&efd_value, &efd_index);
        omega_node.set(&omega_value, &omega_index);
        vs_node.set(&vs_value, &vs_index);
        vuel_node.set(&vuel_value, &vuel_index);

        PhasorDynamics::Exciter::Esdc1a<ScalarT, IdxT> exciter(&bus, data);
        exciter.getSignals().template assignSignalNode<Internal::EFD>(&efd_node);
        exciter.getSignals().template attachSignalNode<External::OMEGA>(&omega_node);
        exciter.getSignals().template attachSignalNode<External::VS>(&vs_node);
        exciter.getSignals().template attachSignalNode<External::VUEL>(&vuel_node);

        bus.allocate();
        exciter.allocate();
        bus.initialize();
        efd_node.init(1.2);
        exciter.initialize();
        exciter.updateTime(0.0, 1.0);

        for (size_t i = 0; i < bus.size(); ++i)
        {
          bus.setVariableIndex(i, static_cast<IdxT>(i + exciter.size()));
          bus.setResidualIndex(i, static_cast<IdxT>(i + exciter.size()));
        }

        bus.evaluateResidual();
        exciter.evaluateResidual();
        exciter.evaluateJacobian();
        exciter.constructCsr();

        auto* model_jacobian = exciter.getCsrJacobian();

        return MapFromCsr(model_jacobian);
      }
#endif
    };
  } // namespace Testing
} // namespace GridKit

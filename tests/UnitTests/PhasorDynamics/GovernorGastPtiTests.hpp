#pragma once

#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>

#include <GridKit/AutomaticDifferentiation/DependencyTracking/Variable.hpp>
#include <GridKit/Definitions.hpp>
#include <GridKit/Model/PhasorDynamics/Governor/GASTPTI/GastPti.hpp>
#include <GridKit/Model/PhasorDynamics/Governor/GASTPTI/GastPtiData.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Testing/TestHelpers.hpp>
#include <GridKit/Testing/Testing.hpp>
#include <GridKit/Utilities/MapFromCsr.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <typename scalar_type, typename index_type>
    class GovernorGastPtiTests
    {
    public:
      using ScalarT = scalar_type;
      using IdxT    = index_type;
      using RealT   = typename PhasorDynamics::Component<ScalarT, IdxT>::RealT;
      using Gov     = PhasorDynamics::Governor::GastPti<ScalarT, IdxT>;
      using Data    = PhasorDynamics::Governor::GastPtiData<RealT, IdxT>;
      using Var     = PhasorDynamics::Governor::GastPtiInternalVariables;
      using Ext     = PhasorDynamics::Governor::GastPtiExternalVariables;
      using Params  = PhasorDynamics::Governor::GastPtiParameters;
      using Mon     = PhasorDynamics::Governor::GastPtiMonitorableVariables;

      GovernorGastPtiTests()  = default;
      ~GovernorGastPtiTests() = default;

      TestOutcome constructor()
      {
        TestStatus success = true;

        Gov model(makeTestData());

        const auto* monitor  = model.getMonitor();
        success             *= (model.size() == static_cast<IdxT>(Var::MAXIMUM));
        success             *= (monitor != nullptr);
        if (monitor != nullptr)
        {
          success *= (!monitor->empty());
        }
        success *= (model.verify() == 0);

        return success.report(__func__);
      }

      TestOutcome zeroInitialResidual()
      {
        TestStatus success = true;

        Gov model(makeTestData());

        PhasorDynamics::SignalNode<ScalarT, IdxT> pmech_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> omega_node;
        const ScalarT                             pmech0 = scalar(kInitialPmech);
        ScalarT                                   pmech_value{0.0};
        ScalarT                                   omega_value = scalar(kInitialOmega);
        IdxT                                      pmech_index = INVALID_INDEX<IdxT>;
        IdxT                                      omega_index = 9;
        pmech_node.set(&pmech_value, &pmech_index);
        omega_node.set(&omega_value, &omega_index);

        model.getSignals().template assignSignalNode<Var::PMECH>(&pmech_node);
        model.getSignals().template attachSignalNode<Ext::OMEGA>(&omega_node);

        success *= (model.allocate() == 0);
        pmech_node.init(pmech0);
        success *= pmech_node.linked();
        success *= (pmech_node.getVariableIndex() == static_cast<IdxT>(Var::PMECH));

        success *= (model.verify() == 0);
        success *= (model.initialize() == 0);
        success *= (model.tagDifferentiable() == 0);
        success *= (model.evaluateResidual() == 0);

        const ScalarT xflow0 = pmech0 + scalar(kDturb) * omega_value;
        const ScalarT vtemp0 = scalar(kAt) + scalar(kKt) * (scalar(kAt) - xflow0);

        success *= isEqual(model.y().getData()[index(Var::XVALVE)], xflow0, scalar(kTolerance));
        success *= isEqual(model.y().getData()[index(Var::XFLOW)], xflow0, scalar(kTolerance));
        success *= isEqual(model.y().getData()[index(Var::XTEMP)], xflow0, scalar(kTolerance));
        success *= isEqual(model.y().getData()[index(Var::VLOAD)], xflow0, scalar(kTolerance));
        success *= isEqual(model.y().getData()[index(Var::VTEMP)], vtemp0, scalar(kTolerance));
        success *= isEqual(model.y().getData()[index(Var::VLV)], xflow0, scalar(kTolerance));
        success *= (model.tag()[index(Var::XVALVE)] == true);
        success *= (model.tag()[index(Var::XFLOW)] == true);
        success *= (model.tag()[index(Var::XTEMP)] == true);

        checkZeroResidual(model, success);

        return success.report(__func__);
      }

      TestOutcome baseConversion()
      {
        TestStatus success = true;

        auto data                      = makeTestData();
        data.parameters[Params::Trate] = static_cast<RealT>(kConversionTrate);
        Gov model(data);
        model.setSystemBase(static_cast<RealT>(kSystemFrequency),
                            static_cast<RealT>(kConversionSystemBase * 1.0e6));

        PhasorDynamics::SignalNode<ScalarT, IdxT> pmech_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> omega_node;
        const ScalarT                             pmech0 = scalar(kConversionInitialPmech);
        ScalarT                                   pmech_value{0.0};
        ScalarT                                   omega_value = scalar(kInitialOmega);
        IdxT                                      pmech_index = INVALID_INDEX<IdxT>;
        IdxT                                      omega_index = 9;
        pmech_node.set(&pmech_value, &pmech_index);
        omega_node.set(&omega_value, &omega_index);

        model.getSignals().template assignSignalNode<Var::PMECH>(&pmech_node);
        model.getSignals().template attachSignalNode<Ext::OMEGA>(&omega_node);

        success *= (model.allocate() == 0);
        pmech_node.init(pmech0);
        success *= (model.verify() == 0);
        success *= (model.initialize() == 0);
        success *= (model.evaluateResidual() == 0);

        const ScalarT pmech_component =
            pmech0 * scalar(kConversionSystemBase / kConversionTrate);
        const ScalarT xflow0 = pmech_component + scalar(kDturb) * omega_value;
        const ScalarT vtemp0 = scalar(kAt) + scalar(kKt) * (scalar(kAt) - xflow0);

        success *= isEqual(model.y().getData()[index(Var::XVALVE)], xflow0, scalar(kTolerance));
        success *= isEqual(model.y().getData()[index(Var::XFLOW)], xflow0, scalar(kTolerance));
        success *= isEqual(model.y().getData()[index(Var::XTEMP)], xflow0, scalar(kTolerance));
        success *= isEqual(model.y().getData()[index(Var::VLOAD)], xflow0, scalar(kTolerance));
        success *= isEqual(model.y().getData()[index(Var::VTEMP)], vtemp0, scalar(kTolerance));
        success *= isEqual(model.y().getData()[index(Var::VLV)], xflow0, scalar(kTolerance));
        success *= isEqual(model.y().getData()[index(Var::PMECH)], pmech0, scalar(kTolerance));

        checkZeroResidual(model, success);

        return success.report(__func__);
      }

      TestOutcome absoluteTolerance()
      {
        TestStatus success = true;

        Gov model(makeTestData());

        success                        *= (model.allocate() == 0);
        success                        *= (model.setAbsoluteTolerance(static_cast<RealT>(1.0e-7)) == 0);
        const auto& absolute_tolerance  = model.absoluteTolerance();
        success                        *= (absolute_tolerance.getSize() == static_cast<IdxT>(Var::MAXIMUM));

        const auto* tolerances = absolute_tolerance.getData();
        for (IdxT i = 0; i < absolute_tolerance.getSize(); ++i)
        {
          success *= isEqual(tolerances[i], scalar(1.0e-7), scalar(kTolerance));
        }

        return success.report(__func__);
      }

      TestOutcome prefSignal()
      {
        TestStatus success = true;

        Gov model(makeTestData());

        PhasorDynamics::SignalNode<ScalarT, IdxT> pmech_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> omega_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> pref_node;
        const ScalarT                             pmech0 = scalar(kInitialPmech);
        ScalarT                                   pmech_value{0.0};
        ScalarT                                   omega_value = scalar(kInitialOmega);
        ScalarT                                   pref_value  = scalar(99.0);
        IdxT                                      pmech_index = INVALID_INDEX<IdxT>;
        IdxT                                      omega_index = 9;
        IdxT                                      pref_index  = 10;
        pmech_node.set(&pmech_value, &pmech_index);
        omega_node.set(&omega_value, &omega_index);
        pref_node.set(&pref_value, &pref_index);

        model.getSignals().template assignSignalNode<Var::PMECH>(&pmech_node);
        model.getSignals().template attachSignalNode<Ext::OMEGA>(&omega_node);
        model.getSignals().template attachSignalNode<Ext::PREF>(&pref_node);

        success *= (model.allocate() == 0);
        pmech_node.init(pmech0);
        success *= (model.verify() == 0);
        success *= (model.initialize() == 0);
        success *= isEqual(pref_node.read(),
                           prefForInitialPoint(pmech0, omega_value),
                           scalar(kTolerance));
        success *= (model.evaluateResidual() == 0);
        checkZeroResidual(model, success);

        pref_value += scalar(kPrefStep);
        success    *= (model.evaluateResidual() == 0);
        success    *= isEqual(model.getResidual().getData()[index(Var::VLOAD)],
                           scalar(kR * kPrefStep),
                           scalar(kTolerance));

        return success.report(__func__);
      }

      TestOutcome prefSignalBaseConversion()
      {
        TestStatus success = true;

        auto data                      = makeTestData();
        data.parameters[Params::Trate] = static_cast<RealT>(kConversionTrate);
        Gov model(data);
        model.setSystemBase(static_cast<RealT>(kSystemFrequency),
                            static_cast<RealT>(kConversionSystemBase * 1.0e6));

        PhasorDynamics::SignalNode<ScalarT, IdxT> pmech_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> omega_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> pref_node;
        const ScalarT                             pmech0 = scalar(kConversionInitialPmech);
        ScalarT                                   pmech_value{0.0};
        ScalarT                                   omega_value = scalar(kInitialOmega);
        ScalarT                                   pref_value  = scalar(99.0);
        IdxT                                      pmech_index = INVALID_INDEX<IdxT>;
        IdxT                                      omega_index = 9;
        IdxT                                      pref_index  = 10;
        pmech_node.set(&pmech_value, &pmech_index);
        omega_node.set(&omega_value, &omega_index);
        pref_node.set(&pref_value, &pref_index);

        model.getSignals().template assignSignalNode<Var::PMECH>(&pmech_node);
        model.getSignals().template attachSignalNode<Ext::OMEGA>(&omega_node);
        model.getSignals().template attachSignalNode<Ext::PREF>(&pref_node);

        success *= (model.allocate() == 0);
        pmech_node.init(pmech0);
        success *= (model.verify() == 0);
        success *= (model.initialize() == 0);

        const ScalarT pmech_component =
            pmech0 * scalar(kConversionSystemBase / kConversionTrate);
        const ScalarT xflow0         = pmech_component + scalar(kDturb) * omega_value;
        const ScalarT pref_component = xflow0 + omega_value / scalar(kR);
        const ScalarT pref_system =
            pref_component * scalar(kConversionTrate / kConversionSystemBase);

        success *= isEqual(pref_node.read(), pref_system, scalar(kTolerance));
        success *= isEqual(model.y().getData()[index(Var::VLOAD)], xflow0, scalar(kTolerance));
        success *= (model.evaluateResidual() == 0);
        checkZeroResidual(model, success);

        pref_value += scalar(kPrefStep);
        success    *= (model.evaluateResidual() == 0);
        success    *= isEqual(model.getResidual().getData()[index(Var::VLOAD)],
                           scalar(kR * kConversionSystemBase / kConversionTrate * kPrefStep),
                           scalar(kTolerance));

        return success.report(__func__);
      }

      TestOutcome residual()
      {
        TestStatus success = true;

        Gov model(makeTestData());

        PhasorDynamics::SignalNode<ScalarT, IdxT> omega_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> pref_node;
        ScalarT                                   omega_value = scalar(kResidualOmega);
        ScalarT                                   pref_value  = scalar(kResidualPref);
        IdxT                                      omega_index = 7;
        IdxT                                      pref_index  = 8;
        omega_node.set(&omega_value, &omega_index);
        pref_node.set(&pref_value, &pref_index);

        model.getSignals().template attachSignalNode<Ext::OMEGA>(&omega_node);
        model.getSignals().template attachSignalNode<Ext::PREF>(&pref_node);

        success *= (model.allocate() == 0);

        const ScalarT xvalve     = scalar(kResidualXvalve);
        const ScalarT xflow      = scalar(kResidualXflow);
        const ScalarT xtemp      = scalar(kResidualXtemp);
        const ScalarT vload      = scalar(kResidualVload);
        const ScalarT vtemp      = scalar(kResidualVtemp);
        const ScalarT vlv        = scalar(kResidualVlv);
        const ScalarT pmech      = scalar(kResidualPmech);
        const ScalarT xvalve_dot = scalar(kResidualXvalveDot);
        const ScalarT xflow_dot  = scalar(kResidualXflowDot);
        const ScalarT xtemp_dot  = scalar(kResidualXtempDot);

        model.y().getData()[index(Var::XVALVE)] = xvalve;
        model.y().getData()[index(Var::XFLOW)]  = xflow;
        model.y().getData()[index(Var::XTEMP)]  = xtemp;
        model.y().getData()[index(Var::VLOAD)]  = vload;
        model.y().getData()[index(Var::VTEMP)]  = vtemp;
        model.y().getData()[index(Var::VLV)]    = vlv;
        model.y().getData()[index(Var::PMECH)]  = pmech;

        model.yp().getData()[index(Var::XVALVE)] = xvalve_dot;
        model.yp().getData()[index(Var::XFLOW)]  = xflow_dot;
        model.yp().getData()[index(Var::XTEMP)]  = xtemp_dot;

        model.y().setDataUpdated();
        model.yp().setDataUpdated();

        success *= (model.verify() == 0);
        success *= (model.evaluateResidual() == 0);

        const ScalarT              valve_target = vlv - xvalve;
        const ScalarT              selected_vlv = vload;
        const std::vector<ScalarT> expected     = {
            -xvalve_dot + valve_target / scalar(kT1),
            -xflow_dot + (-xflow + xvalve) / scalar(kT2),
            -xtemp_dot + (-xtemp + xflow) / scalar(kT3),
            -omega_value + scalar(kR) * (pref_value - vload),
            -vtemp + scalar(kAt) + scalar(kKt) * (scalar(kAt) - xtemp),
            -vlv + selected_vlv,
            -pmech + xflow - scalar(kDturb) * omega_value,
        };

        checkResidual(model, expected, success);

        return success.report(__func__);
      }

      TestOutcome antiWindupLimiter()
      {
        TestStatus success = true;

        Gov model(makeTestData());
        success *= (model.allocate() == 0);

        auto check_valve = [&](ScalarT xvalve, ScalarT vlv, ScalarT expected)
        {
          model.y().setToZero();
          model.yp().setToZero();
          model.y().getData()[index(Var::XVALVE)] = xvalve;
          model.y().getData()[index(Var::VLV)]    = vlv;
          model.y().setDataUpdated();
          success *= (model.evaluateResidual() == 0);
          success *= isEqual(model.getResidual().getData()[index(Var::XVALVE)],
                             expected,
                             scalar(kSmoothTolerance / kT1));
        };

        check_valve(scalar(2.2), scalar(3.2), scalar(0.0));
        check_valve(scalar(2.2), scalar(1.2), scalar(-1.0 / kT1));
        check_valve(scalar(-1.0), scalar(-2.0), scalar(0.0));
        check_valve(scalar(-1.0), scalar(0.0), scalar(1.0 / kT1));

        return success.report(__func__);
      }

      TestOutcome initializationValidation()
      {
        TestStatus success = true;

        auto valve_limited_data = makeTestData();
        Gov  valve_limited_model(valve_limited_data);

        PhasorDynamics::SignalNode<ScalarT, IdxT> valve_limited_pmech_node;
        ScalarT                                   valve_limited_pmech_value{0.0};
        IdxT                                      valve_limited_pmech_index = INVALID_INDEX<IdxT>;
        valve_limited_pmech_node.set(&valve_limited_pmech_value, &valve_limited_pmech_index);

        valve_limited_model.getSignals().template assignSignalNode<Var::PMECH>(
            &valve_limited_pmech_node);

        success *= (valve_limited_model.allocate() == 0);
        valve_limited_pmech_node.init(scalar(kVmax + 0.1));
        success *= (valve_limited_model.initialize() != 0);

        auto temperature_limited_data                   = makeTestData();
        temperature_limited_data.parameters[Params::At] = static_cast<RealT>(0.0);
        Gov temperature_limited_model(temperature_limited_data);

        PhasorDynamics::SignalNode<ScalarT, IdxT> temperature_limited_pmech_node;
        ScalarT                                   temperature_limited_pmech_value{0.0};
        IdxT                                      temperature_limited_pmech_index = INVALID_INDEX<IdxT>;
        temperature_limited_pmech_node.set(&temperature_limited_pmech_value,
                                           &temperature_limited_pmech_index);

        temperature_limited_model.getSignals().template assignSignalNode<Var::PMECH>(
            &temperature_limited_pmech_node);

        success *= (temperature_limited_model.allocate() == 0);
        temperature_limited_pmech_node.init(scalar(0.5));
        success *= (temperature_limited_model.initialize() != 0);

        return success.report(__func__);
      }

      TestOutcome timeConstantMinimum()
      {
        TestStatus success = true;

        auto data                   = makeTestData();
        data.parameters[Params::T1] = static_cast<RealT>(0.0);
        data.parameters[Params::T2] = static_cast<RealT>(kT2);
        data.parameters[Params::T3] = static_cast<RealT>(0.0);

        Gov model(data);
        success *= (model.allocate() == 0);
        success *= (model.tagDifferentiable() == 0);
        success *= (model.tag()[index(Var::XVALVE)] == true);
        success *= (model.tag()[index(Var::XFLOW)] == true);
        success *= (model.tag()[index(Var::XTEMP)] == true);

        const ScalarT xvalve     = scalar(0.7);
        const ScalarT xflow      = scalar(0.6);
        const ScalarT xtemp      = scalar(0.5);
        const ScalarT vlv        = scalar(0.9);
        const ScalarT xflow_dot  = scalar(-0.1);
        const ScalarT unused_dot = scalar(5.0);

        model.y().getData()[index(Var::XVALVE)] = xvalve;
        model.y().getData()[index(Var::XFLOW)]  = xflow;
        model.y().getData()[index(Var::XTEMP)]  = xtemp;
        model.y().getData()[index(Var::VLV)]    = vlv;

        model.yp().getData()[index(Var::XVALVE)] = unused_dot;
        model.yp().getData()[index(Var::XFLOW)]  = xflow_dot;
        model.yp().getData()[index(Var::XTEMP)]  = unused_dot;

        model.y().setDataUpdated();
        model.yp().setDataUpdated();

        success *= (model.evaluateResidual() == 0);
        success *= isEqual(model.getResidual().getData()[index(Var::XVALVE)],
                           -unused_dot + (vlv - xvalve) / scalar(kTimeConstantMinimum),
                           scalar(kTolerance));
        success *= isEqual(model.getResidual().getData()[index(Var::XFLOW)],
                           -xflow_dot + (-xflow + xvalve) / scalar(kT2),
                           scalar(kTolerance));
        success *= isEqual(model.getResidual().getData()[index(Var::XTEMP)],
                           -unused_dot + (-xtemp + xflow) / scalar(kTimeConstantMinimum),
                           scalar(kTolerance));

        return success.report(__func__);
      }

      TestOutcome parameterValidation()
      {
        TestStatus success = true;

        auto missing = makeTestData();
        missing.parameters.erase(Params::R);
        Gov missing_model(missing);
        success *= (missing_model.verify() > 0);

        auto missing_trate = makeTestData();
        missing_trate.parameters.erase(Params::Trate);
        Gov missing_trate_model(missing_trate);
        success *= (missing_trate_model.verify() > 0);

        auto negative_time                   = makeTestData();
        negative_time.parameters[Params::T2] = static_cast<RealT>(-kT2);
        Gov negative_time_model(negative_time);
        success *= (negative_time_model.verify() > 0);

        auto invalid_limits                     = makeTestData();
        invalid_limits.parameters[Params::Vmin] = static_cast<RealT>(kVmax + 0.1);
        invalid_limits.parameters[Params::Vmax] = static_cast<RealT>(kVmax);
        Gov invalid_limits_model(invalid_limits);
        success *= (invalid_limits_model.verify() > 0);

        auto invalid_trate                      = makeTestData();
        invalid_trate.parameters[Params::Trate] = true;
        Gov invalid_trate_model(invalid_trate);
        success *= (invalid_trate_model.verify() > 0);

        auto zero_trate                      = makeTestData();
        zero_trate.parameters[Params::Trate] = static_cast<RealT>(0.0);
        Gov zero_trate_model(zero_trate);
        success *= (zero_trate_model.verify() > 0);

        return success.report(__func__);
      }

      TestOutcome signalValidation()
      {
        TestStatus success = true;

        PhasorDynamics::SignalNode<ScalarT, IdxT> omega_node;
        Gov                                       omega_model(makeTestData());
        omega_model.getSignals().template attachSignalNode<Ext::OMEGA>(&omega_node);
        success *= (omega_model.verify() > 0);

        PhasorDynamics::SignalNode<ScalarT, IdxT> pref_node;
        Gov                                       pref_model(makeTestData());
        pref_model.getSignals().template attachSignalNode<Ext::PREF>(&pref_node);
        success *= (pref_model.verify() > 0);

        return success.report(__func__);
      }

#ifdef GRIDKIT_ENABLE_ENZYME
      TestOutcome jacobian()
      {
        TestStatus success = true;

        auto dependency_tracking_jacobian = dependencyTrackingJacobian();
        auto enzyme_jacobian              = enzymeJacobian();

        success *= (dependency_tracking_jacobian.size() == enzyme_jacobian.size());
        for (size_t i = 0; i < dependency_tracking_jacobian.size(); ++i)
        {
          success *= isEqual(dependency_tracking_jacobian[i], enzyme_jacobian[i], kTolerance);
        }

        return success.report(__func__);
      }
#endif

    private:
      static constexpr RealT kTolerance       = 100.0 * std::numeric_limits<RealT>::epsilon();
      static constexpr RealT kSmoothTolerance = 1.0e-2;

      static constexpr RealT kR                   = 0.05;
      static constexpr RealT kT1                  = 0.4;
      static constexpr RealT kT2                  = 0.5;
      static constexpr RealT kT3                  = 0.25;
      static constexpr RealT kTimeConstantMinimum = 1.0e-3;
      static constexpr RealT kAt                  = 2.0;
      static constexpr RealT kKt                  = 0.3;
      static constexpr RealT kVmax                = 1.2;
      static constexpr RealT kVmin                = 0.0;
      static constexpr RealT kDturb               = 0.1;
      static constexpr RealT kTrate               = 100.0;

      static constexpr RealT kInitialPmech = 0.75;
      static constexpr RealT kInitialOmega = 0.02;
      static constexpr RealT kPrefStep     = 0.1;

      static constexpr RealT kSystemFrequency        = 60.0;
      static constexpr RealT kConversionTrate        = 50.0;
      static constexpr RealT kConversionSystemBase   = 100.0;
      static constexpr RealT kConversionInitialPmech = 0.40;

      static constexpr RealT kResidualOmega     = 0.02;
      static constexpr RealT kResidualPref      = 1.25;
      static constexpr RealT kResidualXvalve    = 0.7;
      static constexpr RealT kResidualXflow     = 0.6;
      static constexpr RealT kResidualXtemp     = 0.5;
      static constexpr RealT kResidualVload     = 0.9;
      static constexpr RealT kResidualVtemp     = 2.5;
      static constexpr RealT kResidualVlv       = 0.8;
      static constexpr RealT kResidualPmech     = 0.55;
      static constexpr RealT kResidualXvalveDot = 0.05;
      static constexpr RealT kResidualXflowDot  = -0.1;
      static constexpr RealT kResidualXtempDot  = 0.2;

      static ScalarT scalar(RealT value)
      {
        return static_cast<ScalarT>(value);
      }

      template <typename value_type>
      static value_type value(RealT value)
      {
        return value_type{value};
      }

      static size_t index(Var variable)
      {
        return static_cast<size_t>(variable);
      }

      template <typename value_type>
      static value_type prefForInitialPoint(const value_type& pmech, const value_type& omega)
      {
        return pmech + value<value_type>(kDturb) * omega + omega / value<value_type>(kR);
      }

      void checkResidual(const Gov&                  model,
                         const std::vector<ScalarT>& expected,
                         TestStatus&                 success) const
      {
        const auto& residual = model.getResidual();
        for (size_t i = 0; i < expected.size(); ++i)
        {
          if (!isEqual(residual.getData()[i], expected[i], scalar(kTolerance)))
          {
            std::cout << "Unexpected GASTPTI residual at index " << i << ": "
                      << residual.getData()[i] << " != " << expected[i] << "\n";
            success = false;
          }
        }
      }

      void checkZeroResidual(const Gov& model, TestStatus& success) const
      {
        std::vector<ScalarT> expected(static_cast<size_t>(Var::MAXIMUM), ScalarT{0});
        checkResidual(model, expected, success);
      }

      Data makeTestData()
      {
        Data data;
        data.device_class          = "GastPti";
        data.disambiguation_string = "gastpti_test";
        data.monitored_variables.insert(Mon::pmech);
        data.monitored_variables.insert(Mon::fuelvalve);

        data.parameters[Params::R]     = static_cast<RealT>(kR);
        data.parameters[Params::T1]    = static_cast<RealT>(kT1);
        data.parameters[Params::T2]    = static_cast<RealT>(kT2);
        data.parameters[Params::T3]    = static_cast<RealT>(kT3);
        data.parameters[Params::At]    = static_cast<RealT>(kAt);
        data.parameters[Params::Kt]    = static_cast<RealT>(kKt);
        data.parameters[Params::Vmax]  = static_cast<RealT>(kVmax);
        data.parameters[Params::Vmin]  = static_cast<RealT>(kVmin);
        data.parameters[Params::Dturb] = static_cast<RealT>(kDturb);
        data.parameters[Params::Trate] = static_cast<RealT>(kTrate);

        return data;
      }

#ifdef GRIDKIT_ENABLE_ENZYME
      using DependencyMap = DependencyTracking::Variable::DependencyMap;

      std::vector<DependencyMap> dependencyTrackingJacobian()
      {
        using ADScalarT = DependencyTracking::Variable;
        using ADGov     = PhasorDynamics::Governor::GastPti<ADScalarT, IdxT>;

        ADGov model(makeTestData());

        PhasorDynamics::SignalNode<ADScalarT, IdxT> omega_node;
        PhasorDynamics::SignalNode<ADScalarT, IdxT> pref_node;
        ADScalarT                                   omega_value{kResidualOmega};
        ADScalarT                                   pref_value{kResidualPref};
        IdxT                                        omega_index = static_cast<IdxT>(Var::MAXIMUM);
        IdxT                                        pref_index  = omega_index + 1;
        omega_node.set(&omega_value, &omega_index);
        pref_node.set(&pref_value, &pref_index);

        model.getSignals().template attachSignalNode<Ext::OMEGA>(&omega_node);
        model.getSignals().template attachSignalNode<Ext::PREF>(&pref_node);
        model.allocate();
        model.updateTime(0.0, 1.0);

        model.y().getData()[index(Var::XVALVE)] = ADScalarT{kResidualXvalve};
        model.y().getData()[index(Var::XFLOW)]  = ADScalarT{kResidualXflow};
        model.y().getData()[index(Var::XTEMP)]  = ADScalarT{kResidualXtemp};
        model.y().getData()[index(Var::VLOAD)]  = ADScalarT{kResidualVload};
        model.y().getData()[index(Var::VTEMP)]  = ADScalarT{kResidualVtemp};
        model.y().getData()[index(Var::VLV)]    = ADScalarT{kResidualVlv};
        model.y().getData()[index(Var::PMECH)]  = ADScalarT{kResidualPmech};

        model.yp().getData()[index(Var::XVALVE)] = ADScalarT{kResidualXvalveDot};
        model.yp().getData()[index(Var::XFLOW)]  = ADScalarT{kResidualXflowDot};
        model.yp().getData()[index(Var::XTEMP)]  = ADScalarT{kResidualXtempDot};

        for (size_t i = 0; i < model.y().getSize(); ++i)
        {
          model.y().getData()[i].setVariableNumber(i);
          model.yp().getData()[i].setVariableNumber(i);
        }
        model.y().setDataUpdated();
        model.yp().setDataUpdated();
        omega_value.setVariableNumber(static_cast<size_t>(omega_index));
        pref_value.setVariableNumber(static_cast<size_t>(pref_index));

        model.evaluateResidual();

        std::vector<DependencyMap> dependencies(model.getResidual().getSize());
        for (size_t i = 0; i < dependencies.size(); ++i)
        {
          dependencies[i] = model.getResidual().getData()[i].getDependencies();
        }

        return dependencies;
      }

      std::vector<DependencyMap> enzymeJacobian()
      {
        Gov model(makeTestData());

        PhasorDynamics::SignalNode<ScalarT, IdxT> omega_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> pref_node;
        ScalarT                                   omega_value = scalar(kResidualOmega);
        ScalarT                                   pref_value  = scalar(kResidualPref);
        IdxT                                      omega_index = static_cast<IdxT>(Var::MAXIMUM);
        IdxT                                      pref_index  = omega_index + 1;
        omega_node.set(&omega_value, &omega_index);
        pref_node.set(&pref_value, &pref_index);

        model.getSignals().template attachSignalNode<Ext::OMEGA>(&omega_node);
        model.getSignals().template attachSignalNode<Ext::PREF>(&pref_node);
        model.allocate();
        model.updateTime(0.0, 1.0);

        model.y().getData()[index(Var::XVALVE)] = scalar(kResidualXvalve);
        model.y().getData()[index(Var::XFLOW)]  = scalar(kResidualXflow);
        model.y().getData()[index(Var::XTEMP)]  = scalar(kResidualXtemp);
        model.y().getData()[index(Var::VLOAD)]  = scalar(kResidualVload);
        model.y().getData()[index(Var::VTEMP)]  = scalar(kResidualVtemp);
        model.y().getData()[index(Var::VLV)]    = scalar(kResidualVlv);
        model.y().getData()[index(Var::PMECH)]  = scalar(kResidualPmech);

        model.yp().getData()[index(Var::XVALVE)] = scalar(kResidualXvalveDot);
        model.yp().getData()[index(Var::XFLOW)]  = scalar(kResidualXflowDot);
        model.yp().getData()[index(Var::XTEMP)]  = scalar(kResidualXtempDot);

        model.y().setDataUpdated();
        model.yp().setDataUpdated();

        model.evaluateResidual();
        model.evaluateJacobian();

        model.constructCsr();
        auto* model_jacobian = model.getCsrJacobian();

        return MapFromCsr(model_jacobian);
      }
#endif
    };
  } // namespace Testing
} // namespace GridKit

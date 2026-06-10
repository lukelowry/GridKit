#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <variant>
#include <vector>

#include <GridKit/AutomaticDifferentiation/DependencyTracking/Variable.hpp>
#include <GridKit/Model/PhasorDynamics/Bus/Bus.hpp>
#include <GridKit/Model/PhasorDynamics/Converter/REECA/Reeca.hpp>
#include <GridKit/Model/PhasorDynamics/Converter/REECA/ReecaData.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNode.hpp>
#include <GridKit/Model/PhasorDynamics/SystemModel.hpp>
#include <GridKit/Model/PhasorDynamics/SystemModelData.hpp>
#include <GridKit/Testing/TestHelpers.hpp>
#include <GridKit/Testing/Testing.hpp>
#include <GridKit/Utilities/MapFromCOO.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <typename scalar_type, typename index_type>
    class ConverterReecaTests
    {
    public:
      using ScalarT = scalar_type;
      using IdxT    = index_type;
      using RealT   = typename PhasorDynamics::Component<ScalarT, IdxT>::RealT;

      static constexpr ScalarT kTol = static_cast<ScalarT>(1.0e-8);

      TestOutcome constructionAndValidation()
      {
        TestStatus success = true;

        PhasorDynamics::Bus<ScalarT, IdxT> bus(1.0, 0.0);

        PhasorDynamics::Converter::Reeca<ScalarT, IdxT> reeca(&bus, makeReecaData());
        success *=
            (reeca.size()
             == static_cast<IdxT>(PhasorDynamics::Converter::ReecaInternalVariables::MAXIMUM));
        success *= (reeca.getMonitor() != nullptr);
        success *= (reeca.verify() == 0);

        auto negative_vdip_reeca = makeReecaData();
        negative_vdip_reeca.parameters[PhasorDynamics::Converter::ReecaParameters::Vdip] =
            static_cast<RealT>(-0.1);
        PhasorDynamics::Converter::Reeca<ScalarT, IdxT> negative_vdip_reeca_model(
            &bus,
            negative_vdip_reeca);
        success *= (negative_vdip_reeca_model.verify() > 0);

        auto bad_reeca_band = makeReecaData();
        bad_reeca_band.parameters[PhasorDynamics::Converter::ReecaParameters::Vdip] =
            static_cast<RealT>(1.2);
        bad_reeca_band.parameters[PhasorDynamics::Converter::ReecaParameters::Vup] =
            static_cast<RealT>(1.2);
        PhasorDynamics::Converter::Reeca<ScalarT, IdxT> bad_reeca_band_model(
            &bus,
            bad_reeca_band);
        success *= (bad_reeca_band_model.verify() > 0);

        auto bad_reeca = makeReecaData();
        bad_reeca.parameters[PhasorDynamics::Converter::ReecaParameters::Imax] =
            static_cast<RealT>(-1.0);
        PhasorDynamics::Converter::Reeca<ScalarT, IdxT> bad_reeca_model(&bus, bad_reeca);
        success *= (bad_reeca_model.verify() > 0);

        return success.report(__func__);
      }

      TestOutcome parameterValidation()
      {
        TestStatus success = true;

        PhasorDynamics::Bus<ScalarT, IdxT> bus(1.0, 0.0);

        auto missing = makeReecaData();
        missing.parameters.erase(Params::Tiq);
        PhasorDynamics::Converter::Reeca<ScalarT, IdxT> missing_model(&bus, missing);
        success *= (missing_model.verify() > 0);

        auto bad_switch                       = makeReecaData();
        bad_switch.parameters[Params::PfFlag] = static_cast<IdxT>(2);
        PhasorDynamics::Converter::Reeca<ScalarT, IdxT> bad_switch_model(&bus, bad_switch);
        success *= (bad_switch_model.verify() > 0);

        auto bad_pflag                      = makeReecaData();
        bad_pflag.parameters[Params::PFlag] = static_cast<IdxT>(2);
        PhasorDynamics::Converter::Reeca<ScalarT, IdxT> bad_pflag_model(&bus, bad_pflag);
        success *= (bad_pflag_model.verify() > 0);

        success *= invalidParameterCase(bus, Params::mva, static_cast<RealT>(0.0));
        success *= invalidParameterCase(bus, Params::Trv, static_cast<RealT>(-0.1));
        success *= invalidParameterCase(bus, Params::Tp, static_cast<RealT>(-0.1));
        success *= invalidParameterCase(bus, Params::Vdip, static_cast<RealT>(-0.1));
        success *= invalidParameterCase(bus, Params::Vup, static_cast<RealT>(0.5));
        success *= invalidParameterCase(bus, Params::dbd1, static_cast<RealT>(0.1));
        success *= invalidParameterCase(bus, Params::dbd2, static_cast<RealT>(-0.1));
        success *= invalidParameterCase(bus, Params::Iql1, static_cast<RealT>(2.0));
        success *= invalidParameterCase(bus, Params::Thld, static_cast<RealT>(1.0));
        success *= invalidParameterCase(bus, Params::Thld2, static_cast<RealT>(1.0));
        success *= invalidParameterCase(bus, Params::Qmin, static_cast<RealT>(2.0));
        success *= invalidParameterCase(bus, Params::Vmin, static_cast<RealT>(2.0));
        success *= invalidParameterCase(bus, Params::Tiq, static_cast<RealT>(0.0));
        success *= invalidParameterCase(bus, Params::Tpord, static_cast<RealT>(0.0));
        success *= invalidParameterCase(bus, Params::dPmin, static_cast<RealT>(0.0));
        success *= invalidParameterCase(bus, Params::dPmax, static_cast<RealT>(0.0));
        success *= invalidParameterCase(bus, Params::Pmin, static_cast<RealT>(2.0));
        success *= invalidParameterCase(bus, Params::Imax, static_cast<RealT>(-0.1));
        success *= invalidParameterCase(bus, Params::Vq2, static_cast<RealT>(0.2));
        success *= invalidParameterCase(bus, Params::Iq1, static_cast<RealT>(-0.1));
        success *= invalidParameterCase(bus, Params::Vp2, static_cast<RealT>(0.2));
        success *= invalidParameterCase(bus, Params::Ip1, static_cast<RealT>(-0.1));

        PhasorDynamics::Converter::Reeca<ScalarT, IdxT> valid_model(&bus, makeReecaData());
        success *= (valid_model.verify() == 0);

        return success.report(__func__);
      }

      TestOutcome reecaSignalsInitializationAndResidual()
      {
        using Var = PhasorDynamics::Converter::ReecaInternalVariables;
        using Ext = PhasorDynamics::Converter::ReecaExternalVariables;

        TestStatus success = true;

        PhasorDynamics::Bus<ScalarT, IdxT> bus(0.8, 0.6);
        bus.allocate();
        bus.initialize();

        ScalarT pe_value{0.75};
        ScalarT qgen_value{0.2};
        ScalarT omega_value{0.01};
        ScalarT iqcmd_value{0.2};
        ScalarT ipcmd_value{0.75};
        IdxT    pe_index    = 20;
        IdxT    qgen_index  = 21;
        IdxT    omega_index = 22;
        IdxT    iqcmd_index = 23;
        IdxT    ipcmd_index = 24;

        PhasorDynamics::SignalNode<ScalarT, IdxT> pe_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> qgen_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> omega_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> iqcmd_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> ipcmd_node;
        pe_node.set(&pe_value, &pe_index);
        qgen_node.set(&qgen_value, &qgen_index);
        omega_node.set(&omega_value, &omega_index);
        iqcmd_node.set(&iqcmd_value, &iqcmd_index);
        ipcmd_node.set(&ipcmd_value, &ipcmd_index);

        auto data                                                          = makeReecaData();
        data.parameters[PhasorDynamics::Converter::ReecaParameters::QFlag] = static_cast<IdxT>(1);
        data.parameters[PhasorDynamics::Converter::ReecaParameters::PFlag] = static_cast<IdxT>(1);
        PhasorDynamics::Converter::Reeca<ScalarT, IdxT> reeca(&bus, data);
        reeca.getSignals().template attachSignalNode<Ext::PE>(&pe_node);
        reeca.getSignals().template attachSignalNode<Ext::QGEN>(&qgen_node);
        reeca.getSignals().template attachSignalNode<Ext::OMEGA>(&omega_node);
        reeca.getSignals().template assignSignalNode<Var::IQCMD>(&iqcmd_node);
        reeca.getSignals().template assignSignalNode<Var::IPCMD>(&ipcmd_node);

        success *= (reeca.allocate() == 0);
        success *= (reeca.verify() == 0);
        iqcmd_node.init(qgen_value);
        ipcmd_node.init(pe_value);
        success *= (reeca.initialize() == 0);
        success *= (reeca.tagDifferentiable() == 0);
        success *= (reeca.evaluateResidual() == 0);

        success *= isEqual(reeca.y()[index(Var::VMEAS)], static_cast<ScalarT>(1.0), kTol);
        success *= isEqual(reeca.y()[index(Var::PMEAS)], pe_value, kTol);
        success *= isEqual(reeca.y()[index(Var::QREF)], qgen_value, kTol);
        success *= isEqual(reeca.y()[index(Var::PORD)], pe_value, kTol);
        success *= isEqual(iqcmd_node.read(), reeca.y()[index(Var::IQCMD)], kTol);
        success *= isEqual(ipcmd_node.read(), reeca.y()[index(Var::IPCMD)], kTol);
        success *= (reeca.tag()[index(Var::VMEAS)] == false);
        success *= (reeca.tag()[index(Var::PMEAS)] == true);

        for (size_t i = 0; i < reeca.getResidual().size(); ++i)
        {
          success *= isEqual(reeca.getResidual()[i], static_cast<ScalarT>(0.0), kTol);
          success *= isEqual(reeca.yp()[i], static_cast<ScalarT>(0.0), kTol);
        }

        return success.report(__func__);
      }

      TestOutcome rejectsHalfConnectedElectricalFeedback()
      {
        using Ext = PhasorDynamics::Converter::ReecaExternalVariables;

        TestStatus success = true;

        PhasorDynamics::Bus<ScalarT, IdxT> bus(1.0, 0.0);

        ScalarT signal_value{0.6};
        IdxT    signal_index = 24;

        PhasorDynamics::SignalNode<ScalarT, IdxT> signal_node;
        signal_node.set(&signal_value, &signal_index);

        {
          PhasorDynamics::Converter::Reeca<ScalarT, IdxT> reeca(&bus, makeReecaData());
          reeca.getSignals().template attachSignalNode<Ext::PE>(&signal_node);
          success *= (reeca.verify() > 0);
        }

        {
          PhasorDynamics::Converter::Reeca<ScalarT, IdxT> reeca(&bus, makeReecaData());
          reeca.getSignals().template attachSignalNode<Ext::QGEN>(&signal_node);
          success *= (reeca.verify() > 0);
        }

        {
          PhasorDynamics::SignalNode<ScalarT, IdxT>       unlinked_node;
          PhasorDynamics::Converter::Reeca<ScalarT, IdxT> reeca(&bus, makeReecaData());
          reeca.getSignals().template attachSignalNode<Ext::OMEGA>(&unlinked_node);
          success *= (reeca.verify() > 0);
        }

        return success.report(__func__);
      }

      TestOutcome reecaCommandSignalInitialization()
      {
        using Var = PhasorDynamics::Converter::ReecaInternalVariables;

        TestStatus success = true;

        PhasorDynamics::Bus<ScalarT, IdxT> bus(1.0, 0.0);
        bus.allocate();
        bus.initialize();

        ScalarT iqcmd_value{0.2};
        ScalarT ipcmd_value{0.6};
        IdxT    iqcmd_index = 22;
        IdxT    ipcmd_index = 23;

        PhasorDynamics::SignalNode<ScalarT, IdxT> iqcmd_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> ipcmd_node;
        iqcmd_node.set(&iqcmd_value, &iqcmd_index);
        ipcmd_node.set(&ipcmd_value, &ipcmd_index);

        PhasorDynamics::Converter::Reeca<ScalarT, IdxT> reeca(&bus, makeReecaData());
        reeca.getSignals().template assignSignalNode<Var::IQCMD>(&iqcmd_node);
        reeca.getSignals().template assignSignalNode<Var::IPCMD>(&ipcmd_node);

        success *= (reeca.allocate() == 0);
        success *= (reeca.verify() == 0);
        iqcmd_node.init(static_cast<ScalarT>(0.2));
        ipcmd_node.init(static_cast<ScalarT>(0.6));
        success *= (reeca.initialize() == 0);
        success *= (reeca.tagDifferentiable() == 0);
        success *= (reeca.evaluateResidual() == 0);

        success *= isEqual(reeca.y()[index(Var::PMEAS)], static_cast<ScalarT>(0.6), kTol);
        success *= isEqual(reeca.y()[index(Var::QREF)], static_cast<ScalarT>(0.2), kTol);
        success *= isEqual(reeca.y()[index(Var::PORD)], static_cast<ScalarT>(0.6), kTol);
        success *= isEqual(reeca.y()[index(Var::IPCMD)], static_cast<ScalarT>(0.6), kTol);
        success *= isEqual(reeca.y()[index(Var::IQCMD)], static_cast<ScalarT>(0.2), kTol);

        for (size_t i = 0; i < reeca.getResidual().size(); ++i)
        {
          success *= isEqual(reeca.getResidual()[i], static_cast<ScalarT>(0.0), kTol);
          success *= isEqual(reeca.yp()[i], static_cast<ScalarT>(0.0), kTol);
        }

        return success.report(__func__);
      }

      TestOutcome reecaElectricalFeedbackUsesMvaBase()
      {
        using Var = PhasorDynamics::Converter::ReecaInternalVariables;
        using Ext = PhasorDynamics::Converter::ReecaExternalVariables;

        TestStatus success = true;

        PhasorDynamics::Bus<ScalarT, IdxT> bus(1.0, 0.0);
        bus.allocate();
        bus.initialize();

        auto data                    = makeReecaData();
        data.parameters[Params::mva] = static_cast<RealT>(50.0);

        ScalarT pe_value{0.25};
        ScalarT qgen_value{0.05};
        ScalarT iqcmd_value{0.1};
        ScalarT ipcmd_value{0.5};
        IdxT    pe_index    = 25;
        IdxT    qgen_index  = 26;
        IdxT    iqcmd_index = 27;
        IdxT    ipcmd_index = 28;

        PhasorDynamics::SignalNode<ScalarT, IdxT> pe_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> qgen_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> iqcmd_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> ipcmd_node;
        pe_node.set(&pe_value, &pe_index);
        qgen_node.set(&qgen_value, &qgen_index);
        iqcmd_node.set(&iqcmd_value, &iqcmd_index);
        ipcmd_node.set(&ipcmd_value, &ipcmd_index);

        PhasorDynamics::Converter::Reeca<ScalarT, IdxT> reeca(&bus, data);
        reeca.getSignals().template attachSignalNode<Ext::PE>(&pe_node);
        reeca.getSignals().template attachSignalNode<Ext::QGEN>(&qgen_node);
        reeca.getSignals().template assignSignalNode<Var::IQCMD>(&iqcmd_node);
        reeca.getSignals().template assignSignalNode<Var::IPCMD>(&ipcmd_node);

        success *= (reeca.allocate() == 0);
        success *= (reeca.verify() == 0);
        iqcmd_node.init(static_cast<ScalarT>(0.1));
        ipcmd_node.init(static_cast<ScalarT>(0.5));
        success *= (reeca.initialize() == 0);
        success *= (reeca.evaluateResidual() == 0);

        success *= isEqual(reeca.y()[index(Var::PMEAS)], static_cast<ScalarT>(0.5), kTol);
        success *= isEqual(reeca.y()[index(Var::QREF)], static_cast<ScalarT>(0.1), kTol);
        success *= isEqual(reeca.y()[index(Var::PORD)], static_cast<ScalarT>(0.5), kTol);
        success *= isEqual(reeca.y()[index(Var::IPCMD)], static_cast<ScalarT>(0.5), kTol);
        success *= isEqual(reeca.y()[index(Var::IQCMD)], static_cast<ScalarT>(0.1), kTol);

        for (size_t i = 0; i < reeca.getResidual().size(); ++i)
        {
          success *= isEqual(reeca.getResidual()[i], static_cast<ScalarT>(0.0), kTol);
        }

        return success.report(__func__);
      }

      TestOutcome reecaReferenceFallbackAtAngle()
      {
        using Var = PhasorDynamics::Converter::ReecaInternalVariables;

        TestStatus success = true;

        PhasorDynamics::Bus<ScalarT, IdxT> bus(0.8, 0.6);
        bus.allocate();
        bus.initialize();

        auto data                       = makeReecaData();
        data.parameters[Params::PfFlag] = static_cast<IdxT>(1);
        data.parameters[Params::Qmin]   = static_cast<RealT>(-2.0);
        data.parameters[Params::Qmax]   = static_cast<RealT>(2.0);
        data.parameters[Params::Pmax]   = static_cast<RealT>(2.0);
        data.parameters[Params::Imax]   = static_cast<RealT>(2.0);

        ScalarT iqcmd_value{-0.2};
        ScalarT ipcmd_value{1.6};
        IdxT    iqcmd_index = 22;
        IdxT    ipcmd_index = 23;

        PhasorDynamics::SignalNode<ScalarT, IdxT> iqcmd_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> ipcmd_node;
        iqcmd_node.set(&iqcmd_value, &iqcmd_index);
        ipcmd_node.set(&ipcmd_value, &ipcmd_index);

        PhasorDynamics::Converter::Reeca<ScalarT, IdxT> reeca(&bus, data);
        reeca.getSignals().template assignSignalNode<Var::IQCMD>(&iqcmd_node);
        reeca.getSignals().template assignSignalNode<Var::IPCMD>(&ipcmd_node);

        success *= (reeca.allocate() == 0);
        success *= (reeca.verify() == 0);
        iqcmd_node.init(static_cast<ScalarT>(-0.2));
        ipcmd_node.init(static_cast<ScalarT>(1.6));
        success *= (reeca.initialize() == 0);
        success *= (reeca.tagDifferentiable() == 0);
        success *= (reeca.evaluateResidual() == 0);

        success *= isEqual(reeca.y()[index(Var::PORD)], static_cast<ScalarT>(1.6), kTol);
        success *= isEqual(reeca.y()[index(Var::QREF)], static_cast<ScalarT>(-0.2), kTol);
        success *= isEqual(reeca.y()[index(Var::IPCMD)], static_cast<ScalarT>(1.6), kTol);
        success *= isEqual(reeca.y()[index(Var::IQCMD)], static_cast<ScalarT>(-0.2), kTol);

        for (size_t i = 0; i < reeca.getResidual().size(); ++i)
        {
          success *= isEqual(reeca.getResidual()[i], static_cast<ScalarT>(0.0), kTol);
          success *= isEqual(reeca.yp()[i], static_cast<ScalarT>(0.0), kTol);
        }

        return success.report(__func__);
      }

      TestOutcome zeroTimeConstantTags()
      {
        using Var = PhasorDynamics::Converter::ReecaInternalVariables;

        TestStatus success = true;

        PhasorDynamics::Bus<ScalarT, IdxT> bus(1.0, 0.0);
        bus.allocate();
        bus.initialize();

        auto data                    = makeReecaData();
        data.parameters[Params::Trv] = static_cast<RealT>(0.0);
        data.parameters[Params::Tp]  = static_cast<RealT>(0.0);

        ScalarT iqcmd_value{0.2};
        ScalarT ipcmd_value{0.6};
        IdxT    iqcmd_index = 22;
        IdxT    ipcmd_index = 23;

        PhasorDynamics::SignalNode<ScalarT, IdxT> iqcmd_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> ipcmd_node;
        iqcmd_node.set(&iqcmd_value, &iqcmd_index);
        ipcmd_node.set(&ipcmd_value, &ipcmd_index);

        PhasorDynamics::Converter::Reeca<ScalarT, IdxT> reeca(&bus, data);
        reeca.getSignals().template assignSignalNode<Var::IQCMD>(&iqcmd_node);
        reeca.getSignals().template assignSignalNode<Var::IPCMD>(&ipcmd_node);

        success *= (reeca.allocate() == 0);
        success *= (reeca.tagDifferentiable() == 0);
        success *= (!reeca.tag()[index(Var::VMEAS)]);
        success *= (!reeca.tag()[index(Var::PMEAS)]);
        success *= (!reeca.tag()[index(Var::VT)]);
        success *= (reeca.tag()[index(Var::XPIQ)]);
        success *= (reeca.tag()[index(Var::XPIV)]);
        success *= (reeca.tag()[index(Var::QV)]);
        success *= (reeca.tag()[index(Var::PORD)]);

        iqcmd_node.init(static_cast<ScalarT>(0.2));
        ipcmd_node.init(static_cast<ScalarT>(0.6));
        success *= (reeca.initialize() == 0);
        success *= (reeca.evaluateResidual() == 0);

        for (size_t i = 0; i < reeca.getResidual().size(); ++i)
        {
          success *= isEqual(reeca.getResidual()[i], static_cast<ScalarT>(0.0), kTol);
        }

        return success.report(__func__);
      }

      TestOutcome outputAvailability()
      {
        TestStatus success = true;

        PhasorDynamics::Bus<ScalarT, IdxT> bus(1.0, 0.0);
        auto                               data = makeReecaData();
        addAllMonitors(data);

        PhasorDynamics::Converter::Reeca<ScalarT, IdxT> reeca(&bus, data);

        success *= (reeca.getMonitor() != nullptr);
        success *= (!reeca.getMonitor()->empty());

        return success.report(__func__);
      }

      TestOutcome priorityInitialization()
      {
        using Var = PhasorDynamics::Converter::ReecaInternalVariables;

        TestStatus success = true;

        PhasorDynamics::Bus<ScalarT, IdxT> bus(1.0, 0.0);
        bus.allocate();
        bus.initialize();

        auto data                       = makeReecaData();
        data.parameters[Params::Pqflag] = static_cast<IdxT>(1);
        data.parameters[Params::Imax]   = static_cast<RealT>(1.2);

        ScalarT iqcmd_value{0.2};
        ScalarT ipcmd_value{0.75};
        IdxT    iqcmd_index = 22;
        IdxT    ipcmd_index = 23;

        PhasorDynamics::SignalNode<ScalarT, IdxT> iqcmd_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> ipcmd_node;
        iqcmd_node.set(&iqcmd_value, &iqcmd_index);
        ipcmd_node.set(&ipcmd_value, &ipcmd_index);

        PhasorDynamics::Converter::Reeca<ScalarT, IdxT> reeca(&bus, data);
        reeca.getSignals().template assignSignalNode<Var::IQCMD>(&iqcmd_node);
        reeca.getSignals().template assignSignalNode<Var::IPCMD>(&ipcmd_node);

        success *= (reeca.allocate() == 0);
        success *= (reeca.verify() == 0);
        iqcmd_node.init(static_cast<ScalarT>(0.2));
        ipcmd_node.init(static_cast<ScalarT>(0.75));
        success *= (reeca.initialize() == 0);

        success *= isEqual(reeca.y()[index(Var::VT)], static_cast<ScalarT>(1.0), kTol);
        success *= isEqual(reeca.y()[index(Var::IPCMD)], static_cast<ScalarT>(0.75), kTol);
        success *= isEqual(reeca.y()[index(Var::IQCMD)], static_cast<ScalarT>(0.2), kTol);
        success *= isEqual(reeca.y()[index(Var::IPCIRC)], static_cast<ScalarT>(1.2), kTol);

        const auto circle =
            reeca.y()[index(Var::IQCIRC)] * reeca.y()[index(Var::IQCIRC)]
            + reeca.y()[index(Var::IPCMD)] * reeca.y()[index(Var::IPCMD)];
        success *= isEqual(circle, static_cast<ScalarT>(1.2 * 1.2), kTol);

        success *= (reeca.evaluateResidual() == 0);
        for (size_t i = 0; i < reeca.getResidual().size(); ++i)
        {
          success *= isEqual(reeca.getResidual()[i], static_cast<ScalarT>(0.0), kTol);
        }

        return success.report(__func__);
      }

      TestOutcome initializesSaturatedStartConsistently()
      {
        using Var = PhasorDynamics::Converter::ReecaInternalVariables;

        TestStatus success = true;

        {
          PhasorDynamics::Bus<ScalarT, IdxT> bus(1.13, 0.0);
          bus.allocate();
          bus.initialize();

          auto data                       = makeReecaData();
          data.parameters[Params::QFlag]  = static_cast<IdxT>(0);
          data.parameters[Params::Pqflag] = static_cast<IdxT>(0);
          data.parameters[Params::Vmin]   = static_cast<RealT>(0.9);
          data.parameters[Params::Vmax]   = static_cast<RealT>(1.05);
          data.parameters[Params::Kvp]    = static_cast<RealT>(10.0);
          data.parameters[Params::Kvi]    = static_cast<RealT>(60.0);
          data.parameters[Params::Vup]    = static_cast<RealT>(99.0);
          data.parameters[Params::Vdip]   = static_cast<RealT>(0.0);
          data.parameters[Params::Imax]   = static_cast<RealT>(1.1);

          ScalarT iqcmd_value{0.15 / 1.13};
          ScalarT ipcmd_value{0.5 / 1.13};
          IdxT    iqcmd_index = 22;
          IdxT    ipcmd_index = 23;

          PhasorDynamics::SignalNode<ScalarT, IdxT> iqcmd_node;
          PhasorDynamics::SignalNode<ScalarT, IdxT> ipcmd_node;
          iqcmd_node.set(&iqcmd_value, &iqcmd_index);
          ipcmd_node.set(&ipcmd_value, &ipcmd_index);

          PhasorDynamics::Converter::Reeca<ScalarT, IdxT> reeca(&bus, data);
          reeca.getSignals().template assignSignalNode<Var::IQCMD>(&iqcmd_node);
          reeca.getSignals().template assignSignalNode<Var::IPCMD>(&ipcmd_node);

          success *= (reeca.allocate() == 0);
          success *= (reeca.verify() == 0);
          iqcmd_node.init(static_cast<ScalarT>(0.15 / 1.13));
          ipcmd_node.init(static_cast<ScalarT>(0.5 / 1.13));
          success *= (reeca.initialize() == 0);

          const ScalarT piv_arg = static_cast<ScalarT>(10.0) * reeca.y()[index(Var::EPIV)]
                                  + reeca.y()[index(Var::XPIV)];
          success *= (piv_arg < -reeca.y()[index(Var::IQMAX)]);

          success *= (reeca.evaluateResidual() == 0);
          for (size_t i = 0; i < reeca.getResidual().size(); ++i)
          {
            success *= isEqual(reeca.getResidual()[i], static_cast<ScalarT>(0.0), kTol);
          }
        }

        {
          PhasorDynamics::Bus<ScalarT, IdxT> bus(0.8, 0.0);
          bus.allocate();
          bus.initialize();

          auto data                     = makeReecaData();
          data.parameters[Params::Vdip] = static_cast<RealT>(0.85);

          PhasorDynamics::Converter::Reeca<ScalarT, IdxT> reeca(&bus, data);
          success *= (reeca.allocate() == 0);
          success *= (reeca.verify() == 0);
          success *= (reeca.initialize() == 1);
        }

        {
          PhasorDynamics::Bus<ScalarT, IdxT> bus(1.0, 0.0);
          bus.allocate();
          bus.initialize();

          auto data                     = makeReecaData();
          data.parameters[Params::Pmax] = static_cast<RealT>(2.0);
          data.parameters[Params::Ip1]  = static_cast<RealT>(1.0);
          data.parameters[Params::Ip2]  = static_cast<RealT>(1.0);
          data.parameters[Params::Ip3]  = static_cast<RealT>(1.0);
          data.parameters[Params::Ip4]  = static_cast<RealT>(1.0);

          ScalarT iqcmd_value{0.0};
          ScalarT ipcmd_value{1.5};
          IdxT    iqcmd_index = 22;
          IdxT    ipcmd_index = 23;

          PhasorDynamics::SignalNode<ScalarT, IdxT> iqcmd_node;
          PhasorDynamics::SignalNode<ScalarT, IdxT> ipcmd_node;
          iqcmd_node.set(&iqcmd_value, &iqcmd_index);
          ipcmd_node.set(&ipcmd_value, &ipcmd_index);

          PhasorDynamics::Converter::Reeca<ScalarT, IdxT> reeca(&bus, data);
          reeca.getSignals().template assignSignalNode<Var::IQCMD>(&iqcmd_node);
          reeca.getSignals().template assignSignalNode<Var::IPCMD>(&ipcmd_node);

          success *= (reeca.allocate() == 0);
          success *= (reeca.verify() == 0);
          iqcmd_node.init(static_cast<ScalarT>(0.0));
          ipcmd_node.init(static_cast<ScalarT>(1.5));
          success *= (reeca.initialize() == 1);
        }

        return success.report(__func__);
      }

#ifdef GRIDKIT_ENABLE_ENZYME
      TestOutcome jacobian()
      {
        TestStatus success = true;

        auto dependency_tracking_jacobian = DependencyTrackingJacobian();
        auto enzyme_jacobian              = EnzymeJacobian();

        success          *= (dependency_tracking_jacobian.size() == enzyme_jacobian.size());
        const auto nrows  = std::min(dependency_tracking_jacobian.size(), enzyme_jacobian.size());
        for (size_t i = 0; i < nrows; ++i)
        {
          const auto dependency_tracking_row =
              pruneJacobianTails(dependency_tracking_jacobian[i], static_cast<RealT>(1.0e-8));
          const auto enzyme_row =
              pruneJacobianTails(enzyme_jacobian[i], static_cast<RealT>(1.0e-8));
          success *= isEqual(dependency_tracking_row, enzyme_row, static_cast<RealT>(1.0e-8));
        }

        return success.report(__func__);
      }
#endif

      TestOutcome jsonParseAndSystemAssembly()
      {
        TestStatus success = true;

        {
          std::istringstream input(commandOnlySystemJson());

          auto data  = PhasorDynamics::parseSystemModelData(input);
          success   *= (data.reeca.size() == 1);
          success   *= (std::get<IdxT>(
                          data.reeca[0].parameters.at(PhasorDynamics::Converter::ReecaParameters::Pqflag))
                      == static_cast<IdxT>(1));
          success   *= (std::get<RealT>(
                          data.reeca[0].parameters.at(PhasorDynamics::Converter::ReecaParameters::mva))
                      == static_cast<RealT>(100.0));

          PhasorDynamics::SystemModel<ScalarT, IdxT> system(data);
          success *= (system.allocate() == 0);
          success *= (system.initialize() == 0);
          success *= (system.evaluateResidual() == 0);
          success *= (system.size() == 27);
        }

        {
          std::istringstream input(signalSourcedSystemJson());

          auto data  = PhasorDynamics::parseSystemModelData(input);
          success   *= (data.reeca.size() == 1);
          success   *= (data.gov.size() == 2);

          PhasorDynamics::SystemModel<ScalarT, IdxT> system(data);
          success *= (system.allocate() == 0);
          success *= (system.initialize() == 0);
          success *= (system.evaluateResidual() == 0);
          for (const auto& residual : system.getResidual())
          {
            success *= std::isfinite(static_cast<RealT>(residual));
          }
        }

        return success.report(__func__);
      }

    private:
      using Params = PhasorDynamics::Converter::ReecaParameters;
      using Mon    = PhasorDynamics::Converter::ReecaMonitorableVariables;

      static size_t index(PhasorDynamics::Converter::ReecaInternalVariables variable)
      {
        return static_cast<size_t>(variable);
      }

      auto makeReecaData() -> PhasorDynamics::Converter::ReecaData<RealT, IdxT>
      {
        PhasorDynamics::Converter::ReecaData<RealT, IdxT> data;
        data.device_class          = "Reeca";
        data.disambiguation_string = "reeca_test";
        data.monitored_variables.insert(Mon::iqcmd);
        data.monitored_variables.insert(Mon::ipcmd);

        data.parameters[Params::mva]    = static_cast<RealT>(100.0);
        data.parameters[Params::PfFlag] = static_cast<IdxT>(0);
        data.parameters[Params::VFlag]  = static_cast<IdxT>(1);
        data.parameters[Params::QFlag]  = static_cast<IdxT>(0);
        data.parameters[Params::PFlag]  = static_cast<IdxT>(0);
        data.parameters[Params::Pqflag] = static_cast<IdxT>(1);
        data.parameters[Params::Trv]    = static_cast<RealT>(0.0);
        data.parameters[Params::Tp]     = static_cast<RealT>(0.02);
        data.parameters[Params::Vdip]   = static_cast<RealT>(0.7);
        data.parameters[Params::Vup]    = static_cast<RealT>(1.2);
        data.parameters[Params::dbd1]   = static_cast<RealT>(-0.01);
        data.parameters[Params::dbd2]   = static_cast<RealT>(0.01);
        data.parameters[Params::kqv]    = static_cast<RealT>(0.0);
        data.parameters[Params::Iql1]   = static_cast<RealT>(-1.0);
        data.parameters[Params::Iqh1]   = static_cast<RealT>(1.0);
        data.parameters[Params::Iqfrz]  = static_cast<RealT>(0.0);
        data.parameters[Params::Thld]   = static_cast<RealT>(0.0);
        data.parameters[Params::Thld2]  = static_cast<RealT>(0.0);
        data.parameters[Params::Qmax]   = static_cast<RealT>(1.0);
        data.parameters[Params::Qmin]   = static_cast<RealT>(-1.0);
        data.parameters[Params::Kqp]    = static_cast<RealT>(1.0);
        data.parameters[Params::Kqi]    = static_cast<RealT>(0.0);
        data.parameters[Params::Vmax]   = static_cast<RealT>(1.2);
        data.parameters[Params::Vmin]   = static_cast<RealT>(0.8);
        data.parameters[Params::Vref1]  = static_cast<RealT>(0.0);
        data.parameters[Params::Kvp]    = static_cast<RealT>(1.0);
        data.parameters[Params::Kvi]    = static_cast<RealT>(0.0);
        data.parameters[Params::Tiq]    = static_cast<RealT>(0.02);
        data.parameters[Params::Tpord]  = static_cast<RealT>(0.02);
        data.parameters[Params::dPmax]  = static_cast<RealT>(1.0);
        data.parameters[Params::dPmin]  = static_cast<RealT>(-1.0);
        data.parameters[Params::Pmax]   = static_cast<RealT>(1.0);
        data.parameters[Params::Pmin]   = static_cast<RealT>(0.0);
        data.parameters[Params::Imax]   = static_cast<RealT>(2.0);
        data.parameters[Params::Vq1]    = static_cast<RealT>(0.2);
        data.parameters[Params::Iq1]    = static_cast<RealT>(2.0);
        data.parameters[Params::Vq2]    = static_cast<RealT>(0.5);
        data.parameters[Params::Iq2]    = static_cast<RealT>(2.0);
        data.parameters[Params::Vq3]    = static_cast<RealT>(0.75);
        data.parameters[Params::Iq3]    = static_cast<RealT>(2.0);
        data.parameters[Params::Vq4]    = static_cast<RealT>(1.0);
        data.parameters[Params::Iq4]    = static_cast<RealT>(2.0);
        data.parameters[Params::Vp1]    = static_cast<RealT>(0.2);
        data.parameters[Params::Ip1]    = static_cast<RealT>(2.0);
        data.parameters[Params::Vp2]    = static_cast<RealT>(0.5);
        data.parameters[Params::Ip2]    = static_cast<RealT>(2.0);
        data.parameters[Params::Vp3]    = static_cast<RealT>(0.75);
        data.parameters[Params::Ip3]    = static_cast<RealT>(2.0);
        data.parameters[Params::Vp4]    = static_cast<RealT>(1.0);
        data.parameters[Params::Ip4]    = static_cast<RealT>(2.0);

        return data;
      }

      void addAllMonitors(PhasorDynamics::Converter::ReecaData<RealT, IdxT>& data)
      {
        data.monitored_variables.insert(Mon::iqcmd);
        data.monitored_variables.insert(Mon::ipcmd);
        data.monitored_variables.insert(Mon::vmeas);
        data.monitored_variables.insert(Mon::pmeas);
        data.monitored_variables.insert(Mon::piq);
        data.monitored_variables.insert(Mon::piv);
        data.monitored_variables.insert(Mon::qv);
        data.monitored_variables.insert(Mon::pord);
        data.monitored_variables.insert(Mon::qref);
        data.monitored_variables.insert(Mon::sdip);
        data.monitored_variables.insert(Mon::iqmax);
        data.monitored_variables.insert(Mon::ipmax);
        data.monitored_variables.insert(Mon::iqv);
        data.monitored_variables.insert(Mon::vqctrl);
        data.monitored_variables.insert(Mon::iqbase);
      }

      bool invalidParameterCase(PhasorDynamics::Bus<ScalarT, IdxT>& bus, Params param, RealT value)
      {
        auto data              = makeReecaData();
        data.parameters[param] = value;
        PhasorDynamics::Converter::Reeca<ScalarT, IdxT> model(&bus, data);
        return model.verify() > 0;
      }

      std::string reecaParamsJson()
      {
        return R"json(
      "params": {
        "mva": 100.0, "PfFlag": 0, "VFlag": 1, "QFlag": 0, "PFlag": 0, "Pqflag": 1,
        "Trv": 0.0, "Tp": 0.02, "Vdip": 0.7, "Vup": 1.2,
        "dbd1": -0.01, "dbd2": 0.01, "kqv": 0.0, "Iql1": -1.0, "Iqh1": 1.0,
        "Iqfrz": 0.0, "Thld": 0.0, "Thld2": 0.0,
        "Qmax": 1.0, "Qmin": -1.0, "Kqp": 1.0, "Kqi": 0.0,
        "Vmax": 1.2, "Vmin": 0.8, "Vref1": 0.0, "Kvp": 1.0, "Kvi": 0.0,
        "Tiq": 0.02, "Tpord": 0.02, "dPmax": 1.0, "dPmin": -1.0,
        "Pmax": 1.0, "Pmin": 0.0, "Imax": 2.0,
        "Vq1": 0.2, "Iq1": 2.0, "Vq2": 0.5, "Iq2": 2.0,
        "Vq3": 0.75, "Iq3": 2.0, "Vq4": 1.0, "Iq4": 2.0,
        "Vp1": 0.2, "Ip1": 2.0, "Vp2": 0.5, "Ip2": 2.0,
        "Vp3": 0.75, "Ip3": 2.0, "Vp4": 1.0, "Ip4": 2.0
      }
)json";
      }

      std::string commandOnlySystemJson()
      {
        return R"json(
{
  "header": {
    "format_version": 0,
    "format_revision": 1,
    "case_name": "renewable electrical control",
    "case_description": "REECA parser test",
    "case_comments": "",
    "freq_base": 60.0,
    "va_base": 100000000.0
  },
  "buses": [
    { "number": 1, "class": "bus", "name": "Bus 1", "init": { "Vr": 1.0, "Vi": 0.0 }, "v_base": 1.0 }
  ],
  "signals": [
    { "signal_id": 12, "name": "Iqcmd" },
    { "signal_id": 13, "name": "Ipcmd" }
  ],
  "devices": [
    {
      "class": "Reeca",
      "ports": { "bus": 1, "iqcmd": 12, "ipcmd": 13 },
      "id": "REE1",
)json" + reecaParamsJson()
               +
               R"json(
    }
  ]
}
)json";
      }

      std::string signalSourcedSystemJson()
      {
        return R"json(
{
  "header": {
    "format_version": 0,
    "format_revision": 1,
    "case_name": "REECA physics",
    "case_description": "REECA parser and system physics smoke test",
    "case_comments": "",
    "freq_base": 60.0,
    "va_base": 100000000.0
  },
  "buses": [
    { "number": 1, "class": "bus", "name": "Bus 1", "init": { "Vr": 1.0, "Vi": 0.0 }, "v_base": 1.0 }
  ],
  "signals": [
    { "signal_id": 10, "name": "pe fixture" },
    { "signal_id": 11, "name": "qgen fixture" },
    { "signal_id": 12, "name": "iqcmd" },
    { "signal_id": 13, "name": "ipcmd" }
  ],
  "devices": [
    {
      "class": "Tgov1",
      "ports": { "pmech": 10 },
      "id": "GP",
      "params": { "R": 0.05, "T1": 0.5, "T2": 2.5, "T3": 7.5, "Pvmax": 1.0, "Pvmin": 0.0, "Dt": 0.0 }
    },
    {
      "class": "Tgov1",
      "ports": { "pmech": 11 },
      "id": "GQ",
      "params": { "R": 0.05, "T1": 0.5, "T2": 2.5, "T3": 7.5, "Pvmax": 1.0, "Pvmin": 0.0, "Dt": 0.0 }
    },
    {
      "class": "Reeca",
      "ports": { "bus": 1, "pe": 10, "qgen": 11, "iqcmd": 12, "ipcmd": 13 },
      "id": "RC",
)json" + reecaParamsJson()
               +
               R"json(
    }
  ]
}
)json";
      }

#ifdef GRIDKIT_ENABLE_ENZYME
      auto makeJacobianData() -> PhasorDynamics::Converter::ReecaData<RealT, IdxT>
      {
        auto data                    = makeReecaData();
        // Distinct VDL limits keep the three linseg coefficients in each of the
        // gq/gp residual rows unequal, which the Enzyme auto-sparsity pass
        // requires to keep structurally identical terms separate.
        data.parameters[Params::Iq1] = static_cast<RealT>(1.20);
        data.parameters[Params::Iq2] = static_cast<RealT>(1.15);
        data.parameters[Params::Iq3] = static_cast<RealT>(1.10);
        data.parameters[Params::Iq4] = static_cast<RealT>(1.05);
        data.parameters[Params::Ip1] = static_cast<RealT>(1.20);
        data.parameters[Params::Ip2] = static_cast<RealT>(1.15);
        data.parameters[Params::Ip3] = static_cast<RealT>(1.10);
        data.parameters[Params::Ip4] = static_cast<RealT>(1.05);
        return data;
      }

      DependencyTracking::Variable::DependencyMap pruneJacobianTails(
          DependencyTracking::Variable::DependencyMap dependencies,
          RealT                                       tolerance)
      {
        for (auto it = dependencies.begin(); it != dependencies.end();)
        {
          if (std::abs(it->second) <= tolerance)
          {
            it = dependencies.erase(it);
          }
          else
          {
            ++it;
          }
        }
        return dependencies;
      }

      std::vector<DependencyTracking::Variable::DependencyMap> DependencyTrackingJacobian()
      {
        using DepVar = DependencyTracking::Variable;
        using Var    = PhasorDynamics::Converter::ReecaInternalVariables;
        using Ext    = PhasorDynamics::Converter::ReecaExternalVariables;

        auto data = makeJacobianData();

        PhasorDynamics::Bus<DepVar, IdxT>              bus(DepVar{0.96}, DepVar{0.28});
        PhasorDynamics::Converter::Reeca<DepVar, IdxT> reeca(&bus, data);

        PhasorDynamics::SignalNode<DepVar, IdxT> pe_node;
        PhasorDynamics::SignalNode<DepVar, IdxT> qgen_node;
        PhasorDynamics::SignalNode<DepVar, IdxT> iqcmd_node;
        PhasorDynamics::SignalNode<DepVar, IdxT> ipcmd_node;
        DepVar                                   pe_value{0.8};
        DepVar                                   qgen_value{0.2};
        DepVar                                   iqcmd_value{0.2};
        DepVar                                   ipcmd_value{0.8};
        IdxT                                     pe_index    = static_cast<IdxT>(reeca.size() + bus.size());
        IdxT                                     qgen_index  = static_cast<IdxT>(reeca.size() + bus.size() + 1);
        IdxT                                     iqcmd_index = static_cast<IdxT>(reeca.size() + bus.size() + 2);
        IdxT                                     ipcmd_index = static_cast<IdxT>(reeca.size() + bus.size() + 3);

        pe_node.set(&pe_value, &pe_index);
        qgen_node.set(&qgen_value, &qgen_index);
        iqcmd_node.set(&iqcmd_value, &iqcmd_index);
        ipcmd_node.set(&ipcmd_value, &ipcmd_index);
        reeca.getSignals().template attachSignalNode<Ext::PE>(&pe_node);
        reeca.getSignals().template attachSignalNode<Ext::QGEN>(&qgen_node);
        reeca.getSignals().template assignSignalNode<Var::IQCMD>(&iqcmd_node);
        reeca.getSignals().template assignSignalNode<Var::IPCMD>(&ipcmd_node);

        bus.allocate();
        reeca.allocate();
        bus.initialize();

        iqcmd_node.init(DepVar{0.2});
        ipcmd_node.init(DepVar{0.8});
        reeca.initialize();

        for (IdxT i = 0; i < reeca.size(); ++i)
        {
          reeca.y()[static_cast<size_t>(i)].setVariableNumber(i);
          reeca.yp()[static_cast<size_t>(i)].setVariableNumber(i);
        }
        for (IdxT i = 0; i < bus.size(); ++i)
        {
          bus.y()[static_cast<size_t>(i)].setVariableNumber(i + reeca.size());
        }
        pe_value.setVariableNumber(pe_index);
        qgen_value.setVariableNumber(qgen_index);

        reeca.evaluateResidual();

        std::vector<DependencyTracking::Variable::DependencyMap> dependencies(
            static_cast<size_t>(reeca.size()));
        for (IdxT i = 0; i < reeca.size(); ++i)
        {
          dependencies[static_cast<size_t>(i)] = reeca.getResidual()[static_cast<size_t>(i)].getDependencies();
        }

        return dependencies;
      }

      std::vector<DependencyTracking::Variable::DependencyMap> EnzymeJacobian()
      {
        using Var = PhasorDynamics::Converter::ReecaInternalVariables;
        using Ext = PhasorDynamics::Converter::ReecaExternalVariables;

        auto data = makeJacobianData();

        PhasorDynamics::Bus<ScalarT, IdxT>              bus(0.96, 0.28);
        PhasorDynamics::Converter::Reeca<ScalarT, IdxT> reeca(&bus, data);

        PhasorDynamics::SignalNode<ScalarT, IdxT> pe_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> qgen_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> iqcmd_node;
        PhasorDynamics::SignalNode<ScalarT, IdxT> ipcmd_node;
        ScalarT                                   pe_value{0.8};
        ScalarT                                   qgen_value{0.2};
        ScalarT                                   iqcmd_value{0.2};
        ScalarT                                   ipcmd_value{0.8};
        IdxT                                      pe_index    = static_cast<IdxT>(reeca.size() + bus.size());
        IdxT                                      qgen_index  = static_cast<IdxT>(reeca.size() + bus.size() + 1);
        IdxT                                      iqcmd_index = static_cast<IdxT>(reeca.size() + bus.size() + 2);
        IdxT                                      ipcmd_index = static_cast<IdxT>(reeca.size() + bus.size() + 3);

        pe_node.set(&pe_value, &pe_index);
        qgen_node.set(&qgen_value, &qgen_index);
        iqcmd_node.set(&iqcmd_value, &iqcmd_index);
        ipcmd_node.set(&ipcmd_value, &ipcmd_index);
        reeca.getSignals().template attachSignalNode<Ext::PE>(&pe_node);
        reeca.getSignals().template attachSignalNode<Ext::QGEN>(&qgen_node);
        reeca.getSignals().template assignSignalNode<Var::IQCMD>(&iqcmd_node);
        reeca.getSignals().template assignSignalNode<Var::IPCMD>(&ipcmd_node);

        bus.allocate();
        reeca.allocate();
        for (IdxT i = 0; i < bus.size(); ++i)
        {
          bus.setVariableIndex(i, i + reeca.size());
          bus.setResidualIndex(i, i + reeca.size());
        }

        bus.initialize();
        iqcmd_node.init(static_cast<ScalarT>(0.2));
        ipcmd_node.init(static_cast<ScalarT>(0.8));
        reeca.initialize();
        reeca.updateTime(0.0, 1.0);

        reeca.evaluateResidual();
        reeca.evaluateJacobian();

        auto model_jacobian = reeca.getJacobian();
        model_jacobian.deduplicate();
        return MapFromCOO(model_jacobian);
      }
#endif
    };
  } // namespace Testing
} // namespace GridKit

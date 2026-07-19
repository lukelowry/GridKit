#pragma once

#include <cstddef>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <GridKit/AutomaticDifferentiation/DependencyTracking/Variable.hpp>
#include <GridKit/Definitions.hpp>
#include <GridKit/Model/PhasorDynamics/Branch/Branch.hpp>
#include <GridKit/Model/PhasorDynamics/Branch/BranchData.hpp>
#include <GridKit/Model/PhasorDynamics/Bus/Bus.hpp>
#include <GridKit/Model/PhasorDynamics/Bus/BusInfinite.hpp>
#include <GridKit/Model/PhasorDynamics/BusFault/BusFault.hpp>
#include <GridKit/Model/PhasorDynamics/ComponentLibrary.hpp>
#include <GridKit/Model/PhasorDynamics/Load/LoadZ/LoadZ.hpp>
#include <GridKit/Model/PhasorDynamics/SystemModel.hpp>
#include <GridKit/Model/PhasorDynamics/SystemModelData.hpp>
#include <GridKit/Testing/TestHelpers.hpp>
#include <GridKit/Testing/Testing.hpp>
#include <GridKit/Utilities/MapFromCsr.hpp>

namespace GridKit
{
  namespace Testing
  {
    using GridKit::PhasorDynamics::BranchBuses;
    using GridKit::PhasorDynamics::BranchParameters;

    template <class ScalarT, typename IdxT>
    class SystemTests
    {
    private:
      using RealT = typename PhasorDynamics::Component<ScalarT, IdxT>::RealT;

    public:
      SystemTests()  = default;
      ~SystemTests() = default;

      /// Constructor, allocation, and initialization checks
      TestOutcome constructor()
      {
        TestStatus success = true;

        PhasorDynamics::SystemModel<ScalarT, IdxT>* system = nullptr;

        // Create an empty system
        system = new PhasorDynamics::SystemModel<ScalarT, IdxT>();

        if (system == nullptr)
        {
          std::cout << "Default constructor failed!\n";
          success = false;
          return success.report(__func__);
        }

        delete system;
        system = nullptr;

        PhasorDynamics::SystemModelData<ScalarT, IdxT> data;

        // Set bus data
        data.bus.resize(2);

        // Bus 0
        data.bus[0].bus_id   = 0;
        data.bus[0].bus_type = PhasorDynamics::BusData<ScalarT, IdxT>::BusType::SLACK;
        data.bus[0].Vr0      = 10.0;
        data.bus[0].Vi0      = 20.0;

        // Bus 1
        data.bus[1].bus_id   = 1;
        data.bus[1].bus_type = PhasorDynamics::BusData<ScalarT, IdxT>::BusType::DEFAULT;
        data.bus[1].Vr0      = 30.0;
        data.bus[1].Vi0      = 40.0;

        // Set branch data
        data.branch.resize(1);

        // Branch 0-1
        data.branch[0].buses[BranchBuses::bus1]        = data.bus[0].bus_id;
        data.branch[0].buses[BranchBuses::bus2]        = data.bus[1].bus_id;
        data.branch[0].parameters[BranchParameters::R] = 2.0;
        data.branch[0].parameters[BranchParameters::X] = 4.0;
        data.branch[0].parameters[BranchParameters::G] = 0.2;
        data.branch[0].parameters[BranchParameters::B] = 1.2;

        // Create an empty system model
        system = new PhasorDynamics::SystemModel<ScalarT, IdxT>(data);
        system->allocate();
        system->initialize();
        system->evaluateResidual();

        // Answer keys
        const ScalarT Ir0{17.0};  ///< Solution: real current entering bus-0
        const ScalarT Ii0{-10.0}; ///< Solution: imaginary current entering bus-0
        const ScalarT Ir1{15.0};  ///< Solution: real current entering bus-1
        const ScalarT Ii1{-20.0}; ///< Solution: imaginary current entering bus-1

        auto* bus0 = system->getBus(0);
        auto* bus1 = system->getBus(1);

        success *= isEqual(bus0->Ir(), Ir0);
        success *= isEqual(bus0->Ii(), Ii0);
        success *= isEqual(bus1->Ir(), Ir1);
        success *= isEqual(bus1->Ii(), Ii1);

        delete system;
        system = nullptr;

        return success.report(__func__);
      }

      TestOutcome composer()
      {
        TestStatus success = true;

        RealT R{2.0}; ///< Branch series resistance
        RealT X{4.0}; ///< Branch series reactance
        RealT G{0.2}; ///< Branch shunt conductance
        RealT B{1.2}; ///< Branch shunt charging

        ScalarT Vr1{10.0}; ///< Bus-1 real voltage
        ScalarT Vi1{20.0}; ///< Bus-1 imaginary voltage
        ScalarT Vr2{30.0}; ///< Bus-2 real voltage
        ScalarT Vi2{40.0}; ///< Bus-2 imaginary voltage

        const ScalarT Ir1{17.0};  ///< Solution: real current entering bus-1
        const ScalarT Ii1{-10.0}; ///< Solution: imaginary current entering bus-1
        const ScalarT Ir2{15.0};  ///< Solution: real current entering bus-2
        const ScalarT Ii2{-20.0}; ///< Solution: imaginary current entering bus-2

        // Create an empty system model
        PhasorDynamics::SystemModel<ScalarT, IdxT> system;

        // Add a bus
        PhasorDynamics::BusInfinite<ScalarT, IdxT> bus1(Vr1, Vi1);
        system.addBus(&bus1);

        // Add a bus
        PhasorDynamics::Bus<ScalarT, IdxT> bus2(Vr2, Vi2);
        system.addBus(&bus2);

        PhasorDynamics::Branch<ScalarT, IdxT> branch(&bus1, &bus2, R, X, G, B);
        system.addComponent(&branch);

        system.allocate();
        system.initialize();
        system.evaluateResidual();

        success *= isEqual(bus1.Ir(), Ir1);
        success *= isEqual(bus1.Ii(), Ii1);
        success *= isEqual(bus2.Ir(), Ir2);
        success *= isEqual(bus2.Ii(), Ii2);

        return success.report(__func__);
      }

      TestOutcome renewableControlChain()
      {
        using namespace PhasorDynamics;
        using namespace PhasorDynamics::Converter;

        TestStatus                   success = true;
        SystemModelData<RealT, IdxT> data;
        constexpr IdxT               bus_id        = 1;
        constexpr IdxT               ipcmd_id      = 10;
        constexpr IdxT               iqcmd_id      = 11;
        constexpr IdxT               ibranchr_id   = 12;
        constexpr IdxT               ibranchi_id   = 13;
        constexpr IdxT               pbranch_id    = 14;
        constexpr IdxT               qbranch_id    = 15;
        constexpr IdxT               qext_id       = 16;
        constexpr IdxT               pext_id       = 17;
        constexpr IdxT               frequency_id  = 18;
        constexpr RealT              p0            = static_cast<RealT>(0.4);
        constexpr RealT              q0            = static_cast<RealT>(0.1);
        constexpr RealT              component_mva = static_cast<RealT>(50.0);
        constexpr RealT              tolerance     = static_cast<RealT>(1.0e-8);

        data.freq_base = static_cast<RealT>(60.0);
        data.va_base   = static_cast<RealT>(100.0e6);
        data.bus.resize(1);
        data.bus[0].bus_id   = bus_id;
        data.bus[0].bus_type = BusData<RealT, IdxT>::BusType::SLACK;
        data.bus[0].Vr0      = static_cast<RealT>(1.0);
        data.bus[0].Vi0      = static_cast<RealT>(0.0);

        for (IdxT signal_id = ipcmd_id; signal_id <= frequency_id; ++signal_id)
        {
          typename SystemModelData<RealT, IdxT>::SignalDataT signal;
          signal.signal_id = signal_id;
          data.signal.push_back(signal);
        }

        RegcaData<RealT, IdxT> regca;
        regca.device_class                                 = "Regca";
        regca.buses[RegcaBuses::bus]                       = bus_id;
        regca.signal_inputs[RegcaSignalInputs::ipcmd]      = ipcmd_id;
        regca.signal_inputs[RegcaSignalInputs::iqcmd]      = iqcmd_id;
        regca.signal_outputs[RegcaSignalOutputs::ibranchr] = ibranchr_id;
        regca.signal_outputs[RegcaSignalOutputs::ibranchi] = ibranchi_id;
        regca.signal_outputs[RegcaSignalOutputs::pbranch]  = pbranch_id;
        regca.signal_outputs[RegcaSignalOutputs::qbranch]  = qbranch_id;
        regca.parameters[RegcaParameters::P0]              = p0;
        regca.parameters[RegcaParameters::Q0]              = q0;
        regca.parameters[RegcaParameters::mva]             = component_mva;
        regca.parameters[RegcaParameters::Tg]              = static_cast<RealT>(0.02);
        regca.parameters[RegcaParameters::TM]              = static_cast<RealT>(0.02);
        regca.parameters[RegcaParameters::Rqmax]           = static_cast<RealT>(999.0);
        regca.parameters[RegcaParameters::Rqmin]           = static_cast<RealT>(-999.0);
        regca.parameters[RegcaParameters::Rpmax]           = static_cast<RealT>(999.0);
        regca.parameters[RegcaParameters::sL]              = true;
        regca.parameters[RegcaParameters::IL1]             = static_cast<RealT>(1.1);
        regca.parameters[RegcaParameters::VL0]             = static_cast<RealT>(0.4);
        regca.parameters[RegcaParameters::VL1]             = static_cast<RealT>(0.9);
        regca.parameters[RegcaParameters::VA0]             = static_cast<RealT>(0.4);
        regca.parameters[RegcaParameters::VA1]             = static_cast<RealT>(0.9);
        regca.parameters[RegcaParameters::Vhvmax]          = static_cast<RealT>(1.2);
        data.regca.push_back(regca);

        ReecbData<RealT, IdxT> reecb;
        reecb.device_class                              = "Reecb";
        reecb.buses[ReecbBuses::bus]                    = bus_id;
        reecb.signal_inputs[ReecbSignalInputs::pe]      = pbranch_id;
        reecb.signal_inputs[ReecbSignalInputs::qgen]    = qbranch_id;
        reecb.signal_inputs[ReecbSignalInputs::qext]    = qext_id;
        reecb.signal_inputs[ReecbSignalInputs::pref]    = pext_id;
        reecb.signal_outputs[ReecbSignalOutputs::iqcmd] = iqcmd_id;
        reecb.signal_outputs[ReecbSignalOutputs::ipcmd] = ipcmd_id;
        reecb.parameters[ReecbParameters::mva]          = component_mva;
        data.reecb.push_back(reecb);

        RepcaData<RealT, IdxT> repca;
        repca.device_class                               = "Repca";
        repca.buses[RepcaBuses::bus]                     = bus_id;
        repca.signal_inputs[RepcaSignalInputs::ibranchr] = ibranchr_id;
        repca.signal_inputs[RepcaSignalInputs::ibranchi] = ibranchi_id;
        repca.signal_inputs[RepcaSignalInputs::pbranch]  = pbranch_id;
        repca.signal_inputs[RepcaSignalInputs::qbranch]  = qbranch_id;
        repca.signal_inputs[RepcaSignalInputs::freq]     = frequency_id;
        repca.signal_outputs[RepcaSignalOutputs::qext]   = qext_id;
        repca.signal_outputs[RepcaSignalOutputs::pext]   = pext_id;
        repca.parameters[RepcaParameters::mva]           = component_mva;
        repca.parameters[RepcaParameters::Freqflag]      = true;
        data.repca.push_back(repca);

        typename SystemModelData<RealT, IdxT>::ConstantSourceT frequency;
        frequency.device_class                                          = "ConstantSignalSource";
        frequency.parameters[ConstantSignalSourceParameters::Sr]        = static_cast<RealT>(1.0);
        frequency.signal_outputs[ConstantSignalSourceSignalOutputs::sr] = frequency_id;
        data.constant_source.push_back(frequency);

        SystemModel<ScalarT, IdxT> system(data);
        auto*                      regca_model = dynamic_cast<Regca<ScalarT, IdxT>*>(system.getComponent(0));
        auto*                      reecb_model = dynamic_cast<Reecb<ScalarT, IdxT>*>(system.getComponent(1));
        auto*                      repca_model = dynamic_cast<Repca<ScalarT, IdxT>*>(system.getComponent(2));

        success *= regca_model != nullptr;
        success *= reecb_model != nullptr;
        success *= repca_model != nullptr;
        if (regca_model == nullptr || reecb_model == nullptr || repca_model == nullptr)
        {
          return success.report(__func__);
        }

        success *= regca_model->getSignals().template isAttached<RegcaExternalVariables::IPCMD>();
        success *= regca_model->getSignals().template isAttached<RegcaExternalVariables::IQCMD>();
        success *= regca_model->getSignals().template getSignalNode<RegcaInternalVariables::IR>()
                   == system.getSignal(ibranchr_id);
        success *= regca_model->getSignals().template getSignalNode<RegcaInternalVariables::II>()
                   == system.getSignal(ibranchi_id);
        success *= regca_model->getSignals().template getSignalNode<RegcaInternalVariables::PBR>()
                   == system.getSignal(pbranch_id);
        success *= regca_model->getSignals().template getSignalNode<RegcaInternalVariables::QBR>()
                   == system.getSignal(qbranch_id);

        success *= reecb_model->getSignals().template isAttached<ReecbExternalVariables::PE>();
        success *= reecb_model->getSignals().template isAttached<ReecbExternalVariables::QGEN>();
        success *= reecb_model->getSignals().template isAttached<ReecbExternalVariables::QEXT>();
        success *= reecb_model->getSignals().template isAttached<ReecbExternalVariables::PREF>();
        success *= reecb_model->getSignals().template getSignalNode<ReecbInternalVariables::IQCMD>()
                   == system.getSignal(iqcmd_id);
        success *= reecb_model->getSignals().template getSignalNode<ReecbInternalVariables::IPCMD>()
                   == system.getSignal(ipcmd_id);

        success *= repca_model->getSignals().template isAttached<RepcaExternalVariables::IBRANCHR>();
        success *= repca_model->getSignals().template isAttached<RepcaExternalVariables::IBRANCHI>();
        success *= repca_model->getSignals().template isAttached<RepcaExternalVariables::PBRANCH>();
        success *= repca_model->getSignals().template isAttached<RepcaExternalVariables::QBRANCH>();
        success *= repca_model->getSignals().template isAttached<RepcaExternalVariables::FREQ>();
        success *= repca_model->getSignals().template getSignalNode<RepcaInternalVariables::QEXT>()
                   == system.getSignal(qext_id);
        success *= repca_model->getSignals().template getSignalNode<RepcaInternalVariables::PEXT>()
                   == system.getSignal(pext_id);

        const int allocation_status      = system.allocate();
        const int initialization_status  = system.initialize();
        success                         *= allocation_status == 0;
        success                         *= initialization_status == 0;
        success                         *= regca_model->getSignals()
                       .template readExternalVariableIndex<RegcaExternalVariables::IPCMD>()
                   == system.getSignal(ipcmd_id)->getVariableIndex();
        success *= regca_model->getSignals()
                       .template readExternalVariableIndex<RegcaExternalVariables::IQCMD>()
                   == system.getSignal(iqcmd_id)->getVariableIndex();
        success *= reecb_model->getSignals()
                       .template readExternalVariableIndex<ReecbExternalVariables::PE>()
                   == system.getSignal(pbranch_id)->getVariableIndex();
        success *= reecb_model->getSignals()
                       .template readExternalVariableIndex<ReecbExternalVariables::QGEN>()
                   == system.getSignal(qbranch_id)->getVariableIndex();
        success *= reecb_model->getSignals()
                       .template readExternalVariableIndex<ReecbExternalVariables::QEXT>()
                   == system.getSignal(qext_id)->getVariableIndex();
        success *= reecb_model->getSignals()
                       .template readExternalVariableIndex<ReecbExternalVariables::PREF>()
                   == system.getSignal(pext_id)->getVariableIndex();
        success *= repca_model->getSignals()
                       .template readExternalVariableIndex<RepcaExternalVariables::IBRANCHR>()
                   == system.getSignal(ibranchr_id)->getVariableIndex();
        success *= repca_model->getSignals()
                       .template readExternalVariableIndex<RepcaExternalVariables::IBRANCHI>()
                   == system.getSignal(ibranchi_id)->getVariableIndex();
        success *= repca_model->getSignals()
                       .template readExternalVariableIndex<RepcaExternalVariables::PBRANCH>()
                   == system.getSignal(pbranch_id)->getVariableIndex();
        success *= repca_model->getSignals()
                       .template readExternalVariableIndex<RepcaExternalVariables::QBRANCH>()
                   == system.getSignal(qbranch_id)->getVariableIndex();
        success *= repca_model->getSignals()
                       .template readExternalVariableIndex<RepcaExternalVariables::FREQ>()
                   == system.getSignal(frequency_id)->getVariableIndex();
        success *= isEqual(
            system.getSignal(ipcmd_id)->read(), static_cast<ScalarT>(p0), tolerance);
        success *= isEqual(
            system.getSignal(iqcmd_id)->read(), static_cast<ScalarT>(q0), tolerance);
        success *= isEqual(
            system.getSignal(pbranch_id)->read(), static_cast<ScalarT>(p0), tolerance);
        success *= isEqual(
            system.getSignal(qbranch_id)->read(), static_cast<ScalarT>(q0), tolerance);
        success *= isEqual(
            system.getSignal(qext_id)->read(), static_cast<ScalarT>(q0), tolerance);
        success *= isEqual(
            system.getSignal(pext_id)->read(), static_cast<ScalarT>(p0), tolerance);
        success *= system.evaluateResidual() == 0;
        for (IdxT i = 0; i < system.getResidual().getSize(); ++i)
        {
          success *= isEqual(
              system.getResidual().getData()[i], static_cast<ScalarT>(0.0), tolerance);
        }
        success *= system.evaluateJacobian() == 0;

        return success.report(__func__);
      }

      TestOutcome stabilizerExciterInitializationOrder()
      {
        using namespace PhasorDynamics;
        using namespace PhasorDynamics::Exciter;
        using namespace PhasorDynamics::Governor;
        using namespace PhasorDynamics::Stabilizer;

        TestStatus                   success = true;
        SystemModelData<RealT, IdxT> data;

        constexpr IdxT bus_id   = 1;
        constexpr IdxT speed_id = 20;
        constexpr IdxT pmech_id = 21;
        constexpr IdxT efd_id   = 22;
        constexpr IdxT vs_id    = 23;
        constexpr IdxT vref_id  = 24;
        constexpr IdxT pref_id  = 25;

        data.freq_base = static_cast<RealT>(60.0);
        data.va_base   = static_cast<RealT>(100.0e6);
        data.bus.resize(1);
        data.bus[0].bus_id   = bus_id;
        data.bus[0].bus_type = BusData<RealT, IdxT>::BusType::SLACK;
        data.bus[0].Vr0      = static_cast<RealT>(1.0);
        data.bus[0].Vi0      = static_cast<RealT>(0.0);

        for (IdxT signal_id = speed_id; signal_id <= pref_id; ++signal_id)
        {
          typename SystemModelData<RealT, IdxT>::SignalDataT signal;
          signal.signal_id = signal_id;
          data.signal.push_back(signal);
        }

        GenrouData<RealT, IdxT> generator;
        generator.device_class                               = "Genrou";
        generator.buses[GenrouBuses::bus]                    = bus_id;
        generator.signal_inputs[GenrouSignalInputs::pmech]   = pmech_id;
        generator.signal_inputs[GenrouSignalInputs::efd]     = efd_id;
        generator.signal_outputs[GenrouSignalOutputs::speed] = speed_id;
        generator.parameters[GenrouParameters::p0]           = static_cast<RealT>(0.3);
        generator.parameters[GenrouParameters::q0]           = static_cast<RealT>(0.0);
        generator.parameters[GenrouParameters::H]            = static_cast<RealT>(3.0);
        generator.parameters[GenrouParameters::D]            = static_cast<RealT>(0.0);
        generator.parameters[GenrouParameters::Ra]           = static_cast<RealT>(0.0);
        generator.parameters[GenrouParameters::Tdop]         = static_cast<RealT>(7.0);
        generator.parameters[GenrouParameters::Tdopp]        = static_cast<RealT>(0.04);
        generator.parameters[GenrouParameters::Tqop]         = static_cast<RealT>(0.75);
        generator.parameters[GenrouParameters::Tqopp]        = static_cast<RealT>(0.05);
        generator.parameters[GenrouParameters::Xd]           = static_cast<RealT>(2.1);
        generator.parameters[GenrouParameters::Xdp]          = static_cast<RealT>(0.2);
        generator.parameters[GenrouParameters::Xdpp]         = static_cast<RealT>(0.18);
        generator.parameters[GenrouParameters::Xq]           = static_cast<RealT>(0.5);
        generator.parameters[GenrouParameters::Xqp]          = static_cast<RealT>(0.5);
        generator.parameters[GenrouParameters::Xqpp]         = static_cast<RealT>(0.18);
        generator.parameters[GenrouParameters::Xl]           = static_cast<RealT>(0.15);
        generator.parameters[GenrouParameters::S10]          = static_cast<RealT>(0.0);
        generator.parameters[GenrouParameters::S12]          = static_cast<RealT>(0.0);
        generator.parameters[GenrouParameters::mva]          = static_cast<RealT>(100.0);
        data.genrou.push_back(generator);

        GastPtiData<RealT, IdxT> governor;
        governor.device_class                                = "GastPti";
        governor.signal_inputs[GastPtiSignalInputs::speed]   = speed_id;
        governor.signal_inputs[GastPtiSignalInputs::pref]    = pref_id;
        governor.signal_outputs[GastPtiSignalOutputs::pmech] = pmech_id;
        governor.parameters[GastPtiParameters::R]            = static_cast<RealT>(0.05);
        governor.parameters[GastPtiParameters::T1]           = static_cast<RealT>(0.4);
        governor.parameters[GastPtiParameters::T2]           = static_cast<RealT>(0.5);
        governor.parameters[GastPtiParameters::T3]           = static_cast<RealT>(0.25);
        governor.parameters[GastPtiParameters::At]           = static_cast<RealT>(0.75);
        governor.parameters[GastPtiParameters::Kt]           = static_cast<RealT>(0.3);
        governor.parameters[GastPtiParameters::Vmax]         = static_cast<RealT>(1.2);
        governor.parameters[GastPtiParameters::Vmin]         = static_cast<RealT>(0.0);
        governor.parameters[GastPtiParameters::Dturb]        = static_cast<RealT>(0.1);
        governor.parameters[GastPtiParameters::Trate]        = static_cast<RealT>(100.0);
        data.gastpti.push_back(governor);

        IeeestData<RealT, IdxT> stabilizer;
        stabilizer.device_class                                = "Ieeest";
        stabilizer.signal_inputs[IeeestSignalInputs::input]    = speed_id;
        stabilizer.signal_outputs[IeeestSignalOutputs::output] = vs_id;
        stabilizer.parameters[IeeestParameters::A1]            = static_cast<RealT>(0.0);
        stabilizer.parameters[IeeestParameters::A2]            = static_cast<RealT>(0.0);
        stabilizer.parameters[IeeestParameters::A3]            = static_cast<RealT>(0.0);
        stabilizer.parameters[IeeestParameters::A4]            = static_cast<RealT>(0.0);
        stabilizer.parameters[IeeestParameters::A5]            = static_cast<RealT>(0.0);
        stabilizer.parameters[IeeestParameters::A6]            = static_cast<RealT>(0.0);
        stabilizer.parameters[IeeestParameters::T1]            = static_cast<RealT>(1.0);
        stabilizer.parameters[IeeestParameters::T2]            = static_cast<RealT>(0.05);
        stabilizer.parameters[IeeestParameters::T3]            = static_cast<RealT>(3.0);
        stabilizer.parameters[IeeestParameters::T4]            = static_cast<RealT>(0.5);
        stabilizer.parameters[IeeestParameters::T5]            = static_cast<RealT>(10.0);
        stabilizer.parameters[IeeestParameters::T6]            = static_cast<RealT>(10.0);
        stabilizer.parameters[IeeestParameters::Ks]            = static_cast<RealT>(1.0);
        stabilizer.parameters[IeeestParameters::Lsmin]         = static_cast<RealT>(0.1);
        stabilizer.parameters[IeeestParameters::Lsmax]         = static_cast<RealT>(0.2);
        stabilizer.parameters[IeeestParameters::Vcl]           = static_cast<RealT>(0.0);
        stabilizer.parameters[IeeestParameters::Vcu]           = static_cast<RealT>(0.0);
        stabilizer.parameters[IeeestParameters::Tdelay]        = static_cast<RealT>(0.0);
        data.stabilizer.push_back(stabilizer);

        Esdc1aData<RealT, IdxT> exciter;
        exciter.device_class                             = "Esdc1a";
        exciter.buses[Esdc1aBuses::bus]                  = bus_id;
        exciter.signal_inputs[Esdc1aSignalInputs::speed] = speed_id;
        exciter.signal_inputs[Esdc1aSignalInputs::vref]  = vref_id;
        exciter.signal_inputs[Esdc1aSignalInputs::vs]    = vs_id;
        exciter.signal_outputs[Esdc1aSignalOutputs::efd] = efd_id;
        exciter.parameters[Esdc1aParameters::Tr]         = static_cast<RealT>(0.0);
        exciter.parameters[Esdc1aParameters::Ka]         = static_cast<RealT>(40.0);
        exciter.parameters[Esdc1aParameters::Ta]         = static_cast<RealT>(0.1);
        exciter.parameters[Esdc1aParameters::Tb]         = static_cast<RealT>(0.0);
        exciter.parameters[Esdc1aParameters::Tc]         = static_cast<RealT>(0.0);
        exciter.parameters[Esdc1aParameters::Vrmax]      = static_cast<RealT>(100.0);
        exciter.parameters[Esdc1aParameters::Vrmin]      = static_cast<RealT>(-100.0);
        exciter.parameters[Esdc1aParameters::Ke]         = static_cast<RealT>(1.0);
        exciter.parameters[Esdc1aParameters::Te]         = static_cast<RealT>(0.5);
        exciter.parameters[Esdc1aParameters::Kf]         = static_cast<RealT>(0.0);
        exciter.parameters[Esdc1aParameters::Tf1]        = static_cast<RealT>(0.7);
        exciter.parameters[Esdc1aParameters::Spdmlt]     = false;
        exciter.parameters[Esdc1aParameters::E1]         = static_cast<RealT>(0.0);
        exciter.parameters[Esdc1aParameters::Se1]        = static_cast<RealT>(0.0);
        exciter.parameters[Esdc1aParameters::E2]         = static_cast<RealT>(0.0);
        exciter.parameters[Esdc1aParameters::Se2]        = static_cast<RealT>(0.0);
        exciter.parameters[Esdc1aParameters::UEL]        = static_cast<IdxT>(2);
        exciter.parameters[Esdc1aParameters::exclim]     = false;
        data.esdc1a.push_back(exciter);

        typename SystemModelData<RealT, IdxT>::ConstantSourceT references;
        references.device_class =
            "ConstantSignalSource";
        references.parameters[ConstantSignalSourceParameters::Sr]        = static_cast<RealT>(0.0);
        references.parameters[ConstantSignalSourceParameters::Si]        = static_cast<RealT>(0.0);
        references.signal_outputs[ConstantSignalSourceSignalOutputs::sr] = pref_id;
        references.signal_outputs[ConstantSignalSourceSignalOutputs::si] = vref_id;
        data.constant_source.push_back(references);

        SystemModel<ScalarT, IdxT> system(data);
        auto*                      generator_model =
            dynamic_cast<Genrou<ScalarT, IdxT>*>(system.getComponent(0));
        auto* governor_model =
            dynamic_cast<GastPti<ScalarT, IdxT>*>(system.getComponent(1));
        auto* stabilizer_model =
            dynamic_cast<Ieeest<ScalarT, IdxT, 0>*>(system.getComponent(2));
        auto* exciter_model =
            dynamic_cast<Esdc1a<ScalarT, IdxT>*>(system.getComponent(3));

        success *= generator_model != nullptr;
        success *= governor_model != nullptr;
        success *= stabilizer_model != nullptr;
        success *= exciter_model != nullptr;
        if (generator_model == nullptr || governor_model == nullptr
            || stabilizer_model == nullptr || exciter_model == nullptr)
        {
          return success.report(__func__);
        }

        success *= generator_model->getSignals()
                       .template getSignalNode<GenrouInternalVariables::OMEGA>()
                   == system.getSignal(speed_id);
        success *= generator_model->getSignals()
                       .template isAttached<GenrouExternalVariables::EFD>();
        success *= governor_model->getSignals()
                       .template isAttached<GastPtiExternalVariables::OMEGA>();
        success *= governor_model->getSignals()
                       .template isAttached<GastPtiExternalVariables::PREF>();
        success *= governor_model->getSignals()
                       .template getSignalNode<GastPtiInternalVariables::PMECH>()
                   == system.getSignal(pmech_id);
        success *= stabilizer_model->getSignals()
                       .template isAttached<IeeestExternalVariables::U>();
        success *= stabilizer_model->getSignals()
                       .template getSignalNode<IeeestInternalVariables<0>::VSS>()
                   == system.getSignal(vs_id);
        success *= exciter_model->getSignals()
                       .template isAttached<Esdc1aExternalVariables::VS>();
        success *= exciter_model->getSignals()
                       .template getSignalNode<Esdc1aInternalVariables::EFD>()
                   == system.getSignal(efd_id);

        success *= system.allocate() == 0;
        success *= system.initialize() == 0;
        success *= governor_model->getSignals()
                       .template readExternalVariableIndex<GastPtiExternalVariables::OMEGA>()
                   == system.getSignal(speed_id)->getVariableIndex();

        const ScalarT efd0 = system.getSignal(efd_id)->read();
        const ScalarT vs0  = system.getSignal(vs_id)->read();
        const ScalarT expected_vref =
            efd0 / static_cast<ScalarT>(40.0) + static_cast<ScalarT>(0.9);
        success *= isEqual(
            vs0,
            static_cast<ScalarT>(0.1),
            static_cast<ScalarT>(1.0e-8));
        success *= isEqual(
            system.getSignal(vref_id)->read(),
            expected_vref,
            static_cast<ScalarT>(1.0e-8));
        success *= system.getSignal(pmech_id)->read() > static_cast<ScalarT>(0.0);

        success *= system.evaluateResidual() == 0;
        for (IdxT i = 0; i < system.getResidual().getSize(); ++i)
        {
          success *= isEqual(
              system.getResidual().getData()[i],
              static_cast<ScalarT>(0.0),
              static_cast<ScalarT>(1.0e-8));
        }
        success *= system.evaluateJacobian() == 0;

        return success.report(__func__);
      }

      TestOutcome reallocateAfterTopologyChange()
      {
        TestStatus success = true;

        PhasorDynamics::SystemModel<ScalarT, IdxT> system;
        PhasorDynamics::Bus<ScalarT, IdxT>         bus1(1.0, 0.0);
        PhasorDynamics::Bus<ScalarT, IdxT>         bus2(1.0, 0.0);
        PhasorDynamics::BusFault<ScalarT, IdxT>    fault(&bus1);

        system.addBus(&bus1);
        system.addComponent(&fault);
        success                    *= system.allocate() == 0;
        const IdxT size_before_bus  = system.size();

        system.addBus(&bus2);
        success *= system.allocate() == 0;
        success *= system.size() == size_before_bus + bus2.size();

#ifdef GRIDKIT_ENABLE_ENZYME
        const auto* jacobian  = system.getCsrJacobian();
        success              *= jacobian != nullptr;

        IdxT nnz_without_branch = 0;
        if (jacobian != nullptr)
        {
          success            *= jacobian->getNumRows() == system.size();
          success            *= jacobian->getNumColumns() == system.size();
          nnz_without_branch  = jacobian->getNnz();
        }
#endif

        PhasorDynamics::Branch<ScalarT, IdxT> branch(&bus1, &bus2);
        system.addComponent(&branch);
        success *= system.allocate() == 0;
        success *= system.evaluateJacobian() == 0;

#ifdef GRIDKIT_ENABLE_ENZYME
        jacobian  = system.getCsrJacobian();
        success  *= jacobian != nullptr;
        if (jacobian != nullptr)
        {
          success *= jacobian->getNumRows() == system.size();
          success *= jacobian->getNumColumns() == system.size();
          success *= jacobian->getNnz() > nnz_without_branch;
        }
#endif

        return success.report(__func__);
      }

      TestOutcome modelVectorsAliasSystemStorage()
      {
        TestStatus success = true;

        PhasorDynamics::SystemModel<ScalarT, IdxT> system;
        PhasorDynamics::Bus<ScalarT, IdxT>         bus1(1.0, 2.0);
        PhasorDynamics::BusInfinite<ScalarT, IdxT> infinite_bus;
        PhasorDynamics::Bus<ScalarT, IdxT>         bus2(3.0, 4.0);
        PhasorDynamics::Branch<ScalarT, IdxT>      branch(&bus1, &bus2);
        PhasorDynamics::LoadZ<ScalarT, IdxT>       load(&bus2, 1.0, 1.0);

        system.addBus(&bus1);
        system.addBus(&infinite_bus);
        system.addBus(&bus2);
        system.addComponent(&branch);
        system.addComponent(&load);

        if (system.allocate() != 0
            || system.setAbsoluteTolerance(1e-4) != 0)
        {
          success = false;
          return success.report(__func__);
        }

        auto checkAlias = [&](auto& system_vector, auto& model_vector, IdxT offset)
        {
          auto*      system_data = system_vector.getData();
          auto*      model_data  = model_vector.getData();
          const auto first       = static_cast<std::size_t>(offset);

          if (!system_data || model_data != system_data + first)
          {
            success = false;
            return;
          }

          success *= system_vector.setToConst(ScalarT{3.0}) == 0;
          success *= isEqual(model_data[0], ScalarT{3.0});

          success *= model_vector.setToConst(ScalarT{4.0}) == 0;
          success *= isEqual(system_data[first], ScalarT{4.0});
        };

        auto checkModel = [&](auto& model, IdxT offset)
        {
          success *= model.getVariableIndex(0) == offset;
          success *= model.getResidualIndex(0) == offset;

          checkAlias(system.y(), model.y(), offset);
          checkAlias(system.yp(), model.yp(), offset);
          checkAlias(system.getResidual(), model.getResidual(), offset);
          checkAlias(system.absoluteTolerance(), model.absoluteTolerance(), offset);
        };

        const IdxT bus2_offset = bus1.size();
        const IdxT load_offset = bus1.size() + bus2.size();
        const auto bus2_first  = static_cast<std::size_t>(bus2_offset);

        auto rebind = [&](auto& model, IdxT offset)
        {
          return model.bind(system.y(),
                            system.yp(),
                            system.getResidual(),
                            system.absoluteTolerance(),
                            offset);
        };

        // Rebinding the same slices is a no-op.
        success *= rebind(bus2, bus2_offset) == 0;
        success *= rebind(load, load_offset) == 0;

        checkModel(bus2, bus2_offset);
        checkModel(load, load_offset);

        // Tags remain model-owned and are collected separately.
        system.tag()[bus2_first]  = true;
        success                  *= system.tagDifferentiable() == 0;
        success                  *= !system.tag()[bus2_first];

        bus2.tag()[0]  = true;
        success       *= !system.tag()[bus2_first];

        return success.report(__func__);
      }

      /**
       * @brief Test for exception when signals are incorrectly configured
       */
      TestOutcome signalError()
      {
        using namespace std::filesystem;
        using namespace GridKit::PhasorDynamics;
        auto input_file = current_path() / "ThreeBusBasicBad.json";
        auto data       = parseSystemModelData(input_file);
        auto sys        = SystemModel<double, size_t>(data);

        TestStatus status{true};
        status *= throws<std::runtime_error>(
            [&]()
            { sys.allocate(); });

        return status.report(__func__);
      }

      /**
       * @brief Test for exception when a child cannot bind to system storage
       */
      TestOutcome allocationError()
      {
        using namespace GridKit::PhasorDynamics;

        TestStatus                 status{true};
        SystemModel<ScalarT, IdxT> system;
        Bus<ScalarT, IdxT>         bus(ScalarT{1.0}, ScalarT{0.0});

        status *= bus.allocate() == 0;
        system.addBus(&bus);
        status *= throws<std::runtime_error>(
            [&]()
            { system.allocate(); });

        return status.report(__func__);
      }

#ifdef GRIDKIT_ENABLE_ENZYME
      TestOutcome jacobian()
      {
        TestStatus success = true;

        PhasorDynamics::SystemModelData<ScalarT, IdxT> data;

        // Set bus data
        data.bus.resize(2);

        // Bus 0
        data.bus[0].bus_id   = 0;
        data.bus[0].bus_type = PhasorDynamics::BusData<ScalarT, IdxT>::BusType::SLACK;
        data.bus[0].Vr0      = 10.0;
        data.bus[0].Vi0      = 20.0;

        // Bus 1
        data.bus[1].bus_id   = 1;
        data.bus[1].bus_type = PhasorDynamics::BusData<ScalarT, IdxT>::BusType::DEFAULT;
        data.bus[1].Vr0      = 30.0;
        data.bus[1].Vi0      = 40.0;

        // Set branch data
        data.branch.resize(1);

        // Branch 0-1
        data.branch[0].buses[BranchBuses::bus1]        = data.bus[0].bus_id;
        data.branch[0].buses[BranchBuses::bus2]        = data.bus[1].bus_id;
        data.branch[0].parameters[BranchParameters::R] = 2.0;
        data.branch[0].parameters[BranchParameters::X] = 4.0;
        data.branch[0].parameters[BranchParameters::G] = 0.2;
        data.branch[0].parameters[BranchParameters::B] = 1.2;

        // Jacobian via DependencyTracking
        std::vector<DependencyTracking::Variable::DependencyMap> dependency_tracking_jacobian = DependencyTrackingJacobian(data);

        // Jacobian via Enzyme
        std::vector<DependencyTracking::Variable::DependencyMap> enzyme_jacobian = EnzymeJacobian(data);

        /// Compare DependencyTracking dependencies to Enzyme's
        for (size_t i = 0; i < dependency_tracking_jacobian.size(); ++i)
        {
          success *= (GridKit::Testing::isEqual(dependency_tracking_jacobian[i], enzyme_jacobian[i]));
        }

        return success.report(__func__);
      }

    private:
      std::vector<DependencyTracking::Variable::DependencyMap> DependencyTrackingJacobian(
          PhasorDynamics::SystemModelData<ScalarT, IdxT> data)
      {
        // Create an empty system model
        PhasorDynamics::SystemModel<DependencyTracking::Variable, IdxT> system(data);

        // Allocate and initialize the system
        system.allocate();
        system.initialize();

        // Set independent variables
        auto* y = system.y().getData();
        for (size_t i = 0; i < system.size(); ++i)
        {
          y[i].setVariableNumber(i);
        }
        system.y().setDataUpdated();

        // Evaluate and get the system residuals
        system.evaluateResidual();
        auto&       residual      = system.getResidual();
        const auto* residual_data = residual.getData();

        // Print the dependencies
        for (size_t i = 0; i < residual.getSize(); ++i)
        {
          std::cout << i << "th residual: ";
          residual_data[i].print(std::cout);
          std::cout << "\n";
        }

        // Extract the dependencies
        std::vector<DependencyTracking::Variable::DependencyMap> dependencies(residual.getSize());
        for (IdxT i = 0; i < residual.getSize(); ++i)
        {
          dependencies[i] = residual_data[i].getDependencies();
        }

        return dependencies;
      }

      std::vector<DependencyTracking::Variable::DependencyMap> EnzymeJacobian(
          PhasorDynamics::SystemModelData<ScalarT, IdxT> data)
      {
        // Create an empty system model
        PhasorDynamics::SystemModel<ScalarT, IdxT> system(data);

        // Allocate and initialize the system
        system.allocate();
        system.initialize();

        // Evaluate and get the system Jacobian
        system.evaluateResidual();
        system.evaluateJacobian();
        GridKit::LinearAlgebra::CsrMatrix<RealT, IdxT>* system_jacobian = system.getCsrJacobian();
        std::cout << "Sparse Csr Matrix: System Jacobian\n";
        system_jacobian->print();

        return GridKit::Testing::MapFromCsr(system_jacobian);
      }
#endif
    };
  } // namespace Testing
} // namespace GridKit

#pragma once

#include <iostream>
#include <vector>

#include <GridKit/Model/EMT/ComponentLibrary.hpp>
#include <GridKit/Model/EMT/Math/RationalApprox/RationalApprox.hpp>
#include <GridKit/Model/EMT/SystemModel.hpp>
#include <GridKit/Testing/Testing.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <class ScalarT, typename IdxT>
    class EMTArchitectureTests
    {
    private:
      using RealT = typename EMT::Component<ScalarT, IdxT>::RealT;

    public:
      TestOutcome busLifecycle()
      {
        TestStatus success = true;

        EMT::BusData<RealT, IdxT> data;
        data.name   = "bus";
        data.bus_id = 7;
        data.vm     = 120.0;
        data.va     = 0.0;

        EMT::Bus<ScalarT, IdxT> bus(data);
        success *= (bus.allocate() == 0);
        success *= (bus.size() == 3);
        success *= (bus.initialize() == 0);
        success *= (bus.tagDifferentiable() == 0);
        success *= bus.tag()[0];
        success *= bus.tag()[1];
        success *= bus.tag()[2];
        success *= (bus.evaluateResidual() == 0);
        success *= (bus.evaluateJacobian() == 0);
        success *= (bus.verify() == 0);
        success *= (bus.getResidual().size() == 3);

        return success.report(__func__);
      }

      TestOutcome componentLifecycle()
      {
        TestStatus success = true;

        EMT::BusData<RealT, IdxT> bus_data;
        bus_data.name = "bus";
        EMT::Bus<ScalarT, IdxT> bus(bus_data);
        bus.allocate();

        EMT::BusData<RealT, IdxT> other_bus_data;
        other_bus_data.name   = "other";
        other_bus_data.bus_id = 1;
        EMT::Bus<ScalarT, IdxT> other_bus(other_bus_data);
        other_bus.allocate();

        EMT::LoadRLData<RealT, IdxT> load_data;
        EMT::LoadRL<ScalarT, IdxT>   load(&bus, load_data);
        success *= (load.allocate() == 0);
        success *= (load.size() == 3);
        success *= (load.initialize() == 0);
        success *= (load.tagDifferentiable() == 0);
        success *= load.tag()[0];
        success *= (load.evaluateResidual() == 0);
        success *= (load.evaluateJacobian() == 0);
        success *= (load.verify() == 0);

        EMT::BranchLumpedConstantData<RealT, IdxT> branch_data;
        EMT::BranchLumpedConstant<ScalarT, IdxT>   branch(&bus, &other_bus, branch_data);
        success *= (branch.allocate() == 0);
        success *= (branch.size() == 3);
        success *= (branch.initialize() == 0);
        success *= (branch.tagDifferentiable() == 0);
        success *= branch.tag()[2];
        success *= (branch.evaluateResidual() == 0);
        success *= (branch.evaluateJacobian() == 0);
        success *= (branch.verify() == 0);

        EMT::VoltageSourceData<RealT, IdxT> source_data;
        EMT::VoltageSource<ScalarT, IdxT>   source(&bus, source_data);
        success *= (source.allocate() == 0);
        success *= (source.size() == 0);
        success *= (source.initialize() == 0);
        success *= (source.tagDifferentiable() == 0);
        success *= (source.evaluateResidual() == 0);
        success *= (source.evaluateJacobian() == 0);
        success *= (source.verify() == 0);

        EMT::BreakerData<RealT, IdxT> breaker_data;
        EMT::Breaker<ScalarT, IdxT>   breaker(&bus, &other_bus, breaker_data);
        success *= (breaker.allocate() == 0);
        success *= (breaker.size() == 3);
        success *= (breaker.initialize() == 0);
        success *= (breaker.tagDifferentiable() == 0);
        success *= !breaker.tag()[0];
        success *= (breaker.evaluateResidual() == 0);
        success *= (breaker.evaluateJacobian() == 0);
        success *= (breaker.verify() == 0);

        EMT::LoadRL<ScalarT, IdxT> missing_bus(nullptr, load_data);
        success *= (missing_bus.verify() == 1);

        return success.report(__func__);
      }

      TestOutcome manualSystemModel()
      {
        TestStatus success = true;

        EMT::BusData<RealT, IdxT> bus1_data;
        bus1_data.name   = "source_bus";
        bus1_data.bus_id = 10;
        bus1_data.vm     = 120.0;

        EMT::BusData<RealT, IdxT> bus2_data;
        bus2_data.name   = "load_bus";
        bus2_data.bus_id = 20;
        bus2_data.vm     = 120.0;

        EMT::Bus<ScalarT, IdxT> bus1(bus1_data);
        EMT::Bus<ScalarT, IdxT> bus2(bus2_data);

        EMT::BranchLumpedConstantData<RealT, IdxT> branch_data;
        EMT::LoadRLData<RealT, IdxT>               load_data;
        EMT::VoltageSourceData<RealT, IdxT>        source_data;
        EMT::BreakerData<RealT, IdxT>              breaker_data;

        EMT::BranchLumpedConstant<ScalarT, IdxT> branch(&bus1, &bus2, branch_data);
        EMT::LoadRL<ScalarT, IdxT>               load(&bus2, load_data);
        EMT::VoltageSource<ScalarT, IdxT>        source(&bus1, source_data);
        EMT::Breaker<ScalarT, IdxT>              breaker(&bus1, &bus2, breaker_data);

        EMT::SystemModel<ScalarT, IdxT> system;
        system.addBus(&bus1);
        system.addBus(&bus2);
        system.addComponent(&branch);
        system.addComponent(&load);
        system.addComponent(&source);
        system.addComponent(&breaker);

        success *= (system.allocate() == 0);
        success *= (system.size() == 15);
        success *= (bus1.getVariableIndex(0) == 0);
        success *= (bus2.getVariableIndex(0) == 3);
        success *= (branch.getVariableIndex(0) == 6);
        success *= (load.getVariableIndex(0) == 9);
        success *= (breaker.getVariableIndex(0) == 12);

        success *= (system.initialize() == 0);
        success *= (system.tagDifferentiable() == 0);
        success *= system.tag()[0];
        success *= system.tag()[6];
        success *= !system.tag()[12];
        success *= (system.evaluateResidual() == 0);
        success *= (system.getResidual().size() == 15);
        success *= (system.evaluateJacobian() == 0);

        return success.report(__func__);
      }

      TestOutcome rationalApproxSkeleton()
      {
        TestStatus success = true;

        EMT::Math::RationalApproxData<RealT, IdxT> data;
        data.dimension         = 3;
        data.d                 = std::vector<RealT>(9, RealT{0.0});
        data.e                 = std::vector<RealT>(9, RealT{0.0});
        data.real_poles        = {-10.0};
        data.real_residues     = std::vector<RealT>(9, RealT{0.0});
        data.pair_real         = {-20.0};
        data.pair_imag         = {120.0};
        data.pair_residue_real = std::vector<RealT>(9, RealT{0.0});
        data.pair_residue_imag = std::vector<RealT>(9, RealT{0.0});

        EMT::Math::RationalApprox<ScalarT, IdxT> rational(data);
        success *= (rational.verify() == 0);
        success *= (rational.dimension() == 3);
        success *= (rational.matrixSize() == 9);
        success *= (rational.realPoleCount() == 1);
        success *= (rational.complexPairCount() == 1);
        success *= (rational.stateCount() == 9);
        success *= (rational.equationCount() == 9);
        success *= !rational.hasDerivativeFeedthrough();

        std::vector<ScalarT> u(3, ScalarT{1.0});
        std::vector<ScalarT> up(3, ScalarT{0.0});
        std::vector<ScalarT> x(static_cast<size_t>(rational.stateCount()), ScalarT{1.0});
        std::vector<ScalarT> xp(static_cast<size_t>(rational.stateCount()), ScalarT{1.0});
        std::vector<ScalarT> f(static_cast<size_t>(rational.stateCount()), ScalarT{1.0});
        std::vector<ScalarT> z(3, ScalarT{1.0});

        success *= (rational.initialize(u.data(), up.data(), x.data(), xp.data()) == 0);
        success *= (x[0] == ScalarT{0.0});
        success *= (xp[8] == ScalarT{0.0});
        success *= (rational.evaluateStateResidual(u.data(), x.data(), xp.data(), f.data()) == 0);
        success *= (f[4] == ScalarT{0.0});
        success *= (rational.evaluateOutput(u.data(), up.data(), x.data(), z.data()) == 0);
        success *= (z[2] == ScalarT{0.0});

        data.real_poles = {RealT{0.0}};
        rational.setData(data);
        success *= (rational.verify() == 1);

        return success.report(__func__);
      }
    };
  } // namespace Testing
} // namespace GridKit

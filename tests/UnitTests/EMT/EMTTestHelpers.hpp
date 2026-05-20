#pragma once

#include <cmath>
#include <complex>
#include <vector>

#include <GridKit/Model/EMT/SystemModelData.hpp>

namespace GridKit
{
  namespace Testing
  {
    template <typename RealT, typename IdxT>
    EMT::LoadRLData<RealT, IdxT> makeLoadRLData(IdxT bus = 1)
    {
      EMT::LoadRLData<RealT, IdxT> data;
      data.device_class                         = "LoadRL";
      data.disambiguation_string                = "load";
      data.parameters[EMT::LoadRLParameters::r] = EMT::PhaseVector<RealT>{2.0, 3.0, 4.0};
      data.parameters[EMT::LoadRLParameters::l] = EMT::PhaseVector<RealT>{0.5, 0.25, 0.125};
      data.ports[EMT::LoadRLPorts::ac]          = bus;
      return data;
    }

    template <typename RealT, typename IdxT>
    EMT::VoltageSourceData<RealT, IdxT> makeVoltageSourceData(IdxT bus = 0)
    {
      const RealT                         pi = std::acos(RealT{-1.0});
      EMT::VoltageSourceData<RealT, IdxT> data;
      data.device_class                                = "VoltageSource";
      data.disambiguation_string                       = "source";
      data.parameters[EMT::VoltageSourceParameters::e] = EMT::PhaseVector<RealT>{120.0, 118.0, 122.0};
      data.parameters[EMT::VoltageSourceParameters::phi] =
          EMT::PhaseVector<RealT>{0.1, -RealT{2.0} * pi / RealT{3.0}, RealT{2.0} * pi / RealT{3.0}};
      data.parameters[EMT::VoltageSourceParameters::r]         = EMT::PhaseVector<RealT>{0.5, 0.75, 1.0};
      data.parameters[EMT::VoltageSourceParameters::frequency] = RealT{60.0};
      data.ports[EMT::VoltageSourcePorts::bus]                 = bus;
      return data;
    }

    template <typename RealT, typename IdxT>
    EMT::BranchLumpedConstantData<RealT, IdxT> makeBranchData(IdxT from = 0, IdxT to = 1)
    {
      EMT::BranchLumpedConstantData<RealT, IdxT> data;
      data.device_class          = "BranchLumpedConstant";
      data.disambiguation_string = "branch";
      data.parameters[EMT::BranchLumpedConstantParameters::r] =
          EMT::PhaseMatrix<RealT>{EMT::PhaseVector<RealT>{0.10, 0.01, 0.02},
                                  EMT::PhaseVector<RealT>{0.01, 0.11, 0.03},
                                  EMT::PhaseVector<RealT>{0.02, 0.03, 0.12}};
      data.parameters[EMT::BranchLumpedConstantParameters::l] =
          EMT::PhaseMatrix<RealT>{EMT::PhaseVector<RealT>{0.50, 0.04, 0.02},
                                  EMT::PhaseVector<RealT>{0.04, 0.55, 0.03},
                                  EMT::PhaseVector<RealT>{0.02, 0.03, 0.60}};
      data.parameters[EMT::BranchLumpedConstantParameters::g] =
          EMT::PhaseMatrix<RealT>{EMT::PhaseVector<RealT>{0.010, 0.001, 0.002},
                                  EMT::PhaseVector<RealT>{0.001, 0.011, 0.003},
                                  EMT::PhaseVector<RealT>{0.002, 0.003, 0.012}};
      data.parameters[EMT::BranchLumpedConstantParameters::c] =
          EMT::PhaseMatrix<RealT>{EMT::PhaseVector<RealT>{0.020, 0.001, 0.002},
                                  EMT::PhaseVector<RealT>{0.001, 0.021, 0.003},
                                  EMT::PhaseVector<RealT>{0.002, 0.003, 0.022}};
      data.parameters[EMT::BranchLumpedConstantParameters::length] = RealT{2.0};
      data.ports[EMT::BranchLumpedConstantPorts::from]             = from;
      data.ports[EMT::BranchLumpedConstantPorts::to]               = to;
      return data;
    }

    template <typename RealT, typename IdxT>
    EMT::SystemModelData<RealT, IdxT> makeTwoBusSystemData()
    {
      EMT::SystemModelData<RealT, IdxT> data;
      data.freq_base = RealT{60.0};

      typename EMT::SystemModelData<RealT, IdxT>::BusDataT source_bus;
      source_bus.name      = "source_bus";
      source_bus.bus_id    = 0;
      source_bus.vm        = 120.0;
      source_bus.va        = 0.0;
      source_bus.freq_base = data.freq_base;

      typename EMT::SystemModelData<RealT, IdxT>::BusDataT load_bus;
      load_bus.name      = "load_bus";
      load_bus.bus_id    = 1;
      load_bus.vm        = 118.0;
      load_bus.va        = -0.05;
      load_bus.freq_base = data.freq_base;

      data.bus = {source_bus, load_bus};
      data.voltage_source.push_back(makeVoltageSourceData<RealT, IdxT>(0));
      data.branch_lumped_constant.push_back(makeBranchData<RealT, IdxT>(0, 1));
      data.load_rl.push_back(makeLoadRLData<RealT, IdxT>(1));
      return data;
    }
  } // namespace Testing
} // namespace GridKit

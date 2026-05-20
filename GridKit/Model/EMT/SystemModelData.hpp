#pragma once

#include <filesystem>
#include <istream>
#include <optional>
#include <string>
#include <vector>

#include <GridKit/Model/EMT/Bus/BusData.hpp>
#include <GridKit/Model/EMT/Components/BranchLumpedConstant/BranchLumpedConstantData.hpp>
#include <GridKit/Model/EMT/Components/Breaker/BreakerData.hpp>
#include <GridKit/Model/EMT/Components/LoadRL/LoadRLData.hpp>
#include <GridKit/Model/EMT/Components/VoltageSource/VoltageSourceData.hpp>
#include <GridKit/Model/VariableMonitor.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename RealT = double, typename IdxT = size_t>
    struct SystemModelData
    {
      using BusDataT                  = BusData<RealT, IdxT>;
      using BranchLumpedConstantDataT = BranchLumpedConstantData<RealT, IdxT>;
      using LoadRLDataT               = LoadRLData<RealT, IdxT>;
      using VoltageSourceDataT        = VoltageSourceData<RealT, IdxT>;
      using BreakerDataT              = BreakerData<RealT, IdxT>;
      using MonitorSinkSpec           = Model::VariableMonitorBase::SinkSpec;

      std::optional<unsigned short> format_version;
      std::string                   case_name;
      std::string                   case_description;

      RealT freq_base{60.0};

      std::vector<BusDataT>                  bus;
      std::vector<BranchLumpedConstantDataT> branch_lumped_constant;
      std::vector<LoadRLDataT>               load_rl;
      std::vector<VoltageSourceDataT>        voltage_source;
      std::vector<BreakerDataT>              breaker;

      std::vector<MonitorSinkSpec> monitor_sink;
    };

    SystemModelData<double, size_t> parseSystemModelData(std::istream&);
    SystemModelData<double, size_t> parseSystemModelData(std::istream&&);
    SystemModelData<double, size_t> parseSystemModelData(const std::filesystem::path&);
    SystemModelData<double, size_t> parseSystemModelData(const std::string&);
  } // namespace EMT
} // namespace GridKit

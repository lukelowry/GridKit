#pragma once

#include <filesystem>
#include <istream>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <GridKit/Model/EMT/Bus/BusData.hpp>
#include <GridKit/Model/EMT/Component/Line/LineLumped/LineLumpedData.hpp>
#include <GridKit/Model/EMT/Component/Load/LoadZ/LoadZData.hpp>
#include <GridKit/Model/EMT/Component/Source/VoltageSource/VoltageSourceData.hpp>
#include <GridKit/Model/EMT/Operators/Rational/VectorFit/VectorFitData.hpp>
#include <GridKit/Model/PhasorDynamics/SignalNode/SignalNodeData.hpp>
#include <GridKit/Model/PhasorDynamics/SignalSource/ConstantSignalSourceData.hpp>
#include <GridKit/Model/VariableMonitor.hpp>

namespace GridKit::EMT
{
  template <typename real_type, typename index_type>
  struct SystemModelData
  {
    using RealT = real_type;
    using IdxT  = index_type;

    using BusDataT           = BusData<RealT, IdxT>;
    using LineLumpedDataT    = LineLumpedData<RealT, IdxT>;
    using LoadZDataT         = LoadZData<RealT, IdxT>;
    using VoltageSourceDataT = VoltageSourceData<RealT, IdxT>;
    using VectorFitDataT     = VectorFitData<RealT, IdxT>;
    using SignalDataT        = PhasorDynamics::SignalNodeData<RealT, IdxT>;
    using ConstantSourceDataT =
        PhasorDynamics::ConstantSignalSourceData<RealT, IdxT>;
    using MonitorSinkSpec = Model::VariableMonitorBase::SinkSpec;

    std::optional<unsigned short> format_version;
    std::optional<unsigned short> format_revision;
    std::string                   case_name;
    std::string                   case_description;
    std::string                   case_comments;

    RealT omega0{2.0 * std::numbers::pi_v<RealT> * 60.0};

    std::vector<BusDataT>            bus;
    std::vector<LineLumpedDataT>     line_lumped;
    std::vector<LoadZDataT>          loadz;
    std::vector<VoltageSourceDataT>  voltage_source;
    std::vector<VectorFitDataT>      vector_fit;
    std::vector<ConstantSourceDataT> constant_source;
    std::vector<SignalDataT>         signal;
    std::vector<MonitorSinkSpec>     monitor_sink;
  };

  SystemModelData<double, std::size_t> parseSystemModelData(std::istream&);
  SystemModelData<double, std::size_t>
  parseSystemModelData(const std::filesystem::path&);
} // namespace GridKit::EMT

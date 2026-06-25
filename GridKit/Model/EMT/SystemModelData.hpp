#pragma once

#include <cstddef>
#include <filesystem>
#include <istream>
#include <optional>
#include <string>
#include <vector>

#include <GridKit/Model/EMT/Component/Bus/BusData.hpp>
#include <GridKit/Model/EMT/Component/Line/LineLumped/LineLumpedData.hpp>
#include <GridKit/Model/EMT/Component/Load/LoadRL/LoadRLData.hpp>
#include <GridKit/Model/EMT/Component/Source/VoltageSource/VoltageSourceData.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename real_type = double, typename index_type = size_t, std::size_t N = 3>
    struct SystemModelData
    {
      using RealT              = real_type;
      using IdxT               = index_type;
      using BusDataT           = BusData<RealT, IdxT, N>;
      using VoltageSourceDataT = VoltageSourceData<RealT, IdxT, N>;
      using LoadRLDataT        = LoadRLData<RealT, IdxT>;
      using LineLumpedDataT    = LineLumpedData<RealT, IdxT, N>;

      std::optional<unsigned short> format_version;
      std::optional<unsigned short> format_revision;
      std::optional<std::string>    case_date_time;

      std::string case_name;
      std::string case_description;
      std::string case_comments;

      std::vector<BusDataT>           bus;
      std::vector<VoltageSourceDataT> voltage_source;
      std::vector<LoadRLDataT>        load_rl;
      std::vector<LineLumpedDataT>    line_lumped;
    };

    SystemModelData<double, size_t, 3> parseSystemModelData(std::istream&);
    SystemModelData<double, size_t, 3> parseSystemModelData(std::istream&&);
    SystemModelData<double, size_t, 3> parseSystemModelData(const std::filesystem::path&);
    SystemModelData<double, size_t, 3> parseSystemModelData(const std::string&);
  } // namespace EMT
} // namespace GridKit

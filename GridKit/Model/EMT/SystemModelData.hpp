/**
 * @file SystemModelData.hpp
 * @brief Data-only EMT system model representation and JSON parser entry points.
 */

#pragma once

#include <filesystem>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

#include <GridKit/Model/EMT/Bus/BusData.hpp>
#include <GridKit/Model/EMT/Line/LineData.hpp>
#include <GridKit/Model/EMT/Load/ShuntLoadData.hpp>
#include <GridKit/Model/EMT/Source/VoltageSource/VoltageSourceData.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename RealT = double, typename IdxT = size_t>
    struct SystemModelData
    {
      using BusDataT           = BusData<RealT, IdxT>;
      using LineDataT          = LineData<RealT, IdxT>;
      using ShuntLoadDataT     = ShuntLoadData<RealT, IdxT>;
      using VoltageSourceDataT = VoltageSourceData<RealT, IdxT>;

      struct BusSpec
      {
        IdxT     number{0};
        BusDataT data;
      };

      struct LineSpec
      {
        std::string id;
        IdxT        bus1{0};
        IdxT        bus2{0};
        LineDataT   data;
      };

      struct ShuntLoadSpec
      {
        std::string    id;
        IdxT           bus{0};
        ShuntLoadDataT data;
      };

      struct VoltageSourceSpec
      {
        std::string        id;
        IdxT               bus{0};
        VoltageSourceDataT data;
      };

      std::optional<unsigned short> format_version;
      std::optional<unsigned short> format_revision;
      std::string                   case_name;
      std::string                   case_date_time;
      std::string                   case_description;
      std::string                   case_comments;
      RealT                         freq_base{60.0};
      RealT                         va_base{100.0e6};

      std::vector<BusSpec>           bus;
      std::vector<LineSpec>          line;
      std::vector<ShuntLoadSpec>     shunt_load;
      std::vector<VoltageSourceSpec> voltage_source;
    };

    SystemModelData<double, size_t> parseSystemModelData(std::istream& input);
    SystemModelData<double, size_t> parseSystemModelData(const std::filesystem::path& input_file);
    SystemModelData<double, size_t> parseSystemModelDataFromString(const std::string& input);

  } // namespace EMT
} // namespace GridKit

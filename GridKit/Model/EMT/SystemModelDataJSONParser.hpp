#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Component/Bus/BusDataJSONParser.hpp>
#include <GridKit/Model/EMT/Component/Line/LineLumped/LineLumpedDataJSONParser.hpp>
#include <GridKit/Model/EMT/Component/Load/LoadRL/LoadRLDataJSONParser.hpp>
#include <GridKit/Model/EMT/Component/Source/VoltageSource/VoltageSourceDataJSONParser.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>

namespace GridKit
{
  namespace EMT
  {
    using json = nlohmann::json;

    template <typename RealT = double, typename IdxT = size_t, std::size_t N = 3>
    void from_json(const json& j, SystemModelData<RealT, IdxT, N>& data)
    {
      const auto& header = j.at("header");

      data.format_version.reset();
      data.format_revision.reset();
      data.case_date_time.reset();

      if (header.contains("format_version"))
      {
        header.at("format_version").get_to(data.format_version);
      }

      if (header.contains("format_revision"))
      {
        header.at("format_revision").get_to(data.format_revision);
      }

      if (header.contains("case_date_time"))
      {
        header.at("case_date_time").get_to(data.case_date_time);
      }

      header.at("case_name").get_to(data.case_name);
      header.at("case_description").get_to(data.case_description);
      header.at("case_comments").get_to(data.case_comments);

      data.bus.clear();
      data.voltage_source.clear();
      data.load_rl.clear();
      data.line_lumped.clear();

      j.at("buses").get_to(data.bus);

      for (const auto& raw_component : j.at("devices"))
      {
        const auto kind = raw_component.at("class").template get<std::string>();

        if (kind == "VoltageSource")
        {
          typename SystemModelData<RealT, IdxT, N>::VoltageSourceDataT component;
          raw_component.get_to(component);
          data.voltage_source.push_back(component);
        }
        else if (kind == "LoadRL")
        {
          typename SystemModelData<RealT, IdxT, N>::LoadRLDataT component;
          raw_component.get_to(component);
          data.load_rl.push_back(component);
        }
        else if (kind == "LineLumped")
        {
          typename SystemModelData<RealT, IdxT, N>::LineLumpedDataT component;
          raw_component.get_to(component);
          data.line_lumped.push_back(component);
        }
        else
        {
          throw std::invalid_argument("EMT SystemModelData: invalid component class \"" + kind + "\"");
        }
      }
    }
  } // namespace EMT
} // namespace GridKit

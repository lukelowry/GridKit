#pragma once

#include <magic_enum/magic_enum.hpp>
#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Bus/BusData.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace EMT
  {
    using json = nlohmann::json;
    using Log  = ::GridKit::Utilities::Logger;

    template <typename RealT, typename IdxT>
    void from_json(const json& j, BusData<RealT, IdxT>& bd)
    {
      j.at("name").get_to(bd.name);

      if (j.contains("number"))
      {
        j.at("number").get_to(bd.bus_id);
      }

      if (j.contains("init"))
      {
        const auto& init = j.at("init");
        if (init.contains("vm"))
        {
          init.at("vm").get_to(bd.vm);
        }
        if (init.contains("va"))
        {
          init.at("va").get_to(bd.va);
        }
      }

      if (j.contains("freq_base"))
      {
        j.at("freq_base").get_to(bd.freq_base);
      }

      if (j.contains("mon"))
      {
        using MonitorableVariables = typename BusData<RealT, IdxT>::MonitorableVariables;
        for (const auto& raw_monitored_variable : j.at("mon"))
        {
          auto var_name  = raw_monitored_variable.template get<std::string>();
          auto monitored = magic_enum::enum_cast<MonitorableVariables>(var_name, magic_enum::case_insensitive);
          if (monitored.has_value())
          {
            bd.monitored_variables.insert(monitored.value());
          }
          else
          {
            Log::error() << "\n\tInvalid EMT bus monitored variable: \"" << var_name << "\"." << std::endl;
          }
        }
      }
    }
  } // namespace EMT
} // namespace GridKit

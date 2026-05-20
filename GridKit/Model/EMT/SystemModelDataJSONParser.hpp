#pragma once

#include <map>
#include <sstream>
#include <stdexcept>

#include <magic_enum/magic_enum.hpp>
#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/Bus/BusDataJSONParser.hpp>
#include <GridKit/Model/EMT/ComponentDataJSONParser.hpp>
#include <GridKit/Model/EMT/SystemModelData.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace EMT
  {
    using json          = nlohmann::json;
    using Log           = ::GridKit::Utilities::Logger;
    using MonitorFormat = ::GridKit::Model::VariableMonitorFormat;

    template <typename RealT, typename IdxT, typename DataT>
    void parseNamedPortComponent(const json&                        j,
                                 DataT&                             data,
                                 const std::map<std::string, IdxT>& bus_name_to_id)
    {
      using Parameters           = typename DataT::Parameters;
      using Ports                = typename DataT::Ports;
      using MonitorableVariables = typename DataT::MonitorableVariables;
      using BaseDataT            = ComponentData<RealT, IdxT, Parameters, Ports, MonitorableVariables>;

      parseComponentDataCommon(j, static_cast<BaseDataT&>(data));

      for (const auto& raw_port : j.at("ports").items())
      {
        auto key = magic_enum::enum_cast<Ports>(raw_port.key(), magic_enum::case_insensitive);
        if (!key.has_value())
        {
          Log::error() << "\n\tInvalid EMT component port: \"" << raw_port.key() << "\"." << std::endl;
          continue;
        }

        if (raw_port.value().is_string())
        {
          const auto bus_name     = raw_port.value().template get<std::string>();
          data.ports[key.value()] = bus_name_to_id.at(bus_name);
        }
        else
        {
          data.ports[key.value()] = raw_port.value().template get<IdxT>();
        }
      }
    }

    template <typename RealT = double, typename IdxT = size_t>
    void from_json(const json& j, SystemModelData<RealT, IdxT>& sm)
    {
      auto header = j.at("header");
      if (header.contains("format_version"))
      {
        header.at("format_version").get_to(sm.format_version);
      }
      if (header.contains("case_name"))
      {
        header.at("case_name").get_to(sm.case_name);
      }
      if (header.contains("description"))
      {
        header.at("description").get_to(sm.case_description);
      }
      if (header.contains("case_description"))
      {
        header.at("case_description").get_to(sm.case_description);
      }
      if (header.contains("freq_base"))
      {
        header.at("freq_base").get_to(sm.freq_base);
      }

      std::map<std::string, IdxT> bus_name_to_id;
      IdxT                        next_bus_id = 0;
      for (const auto& raw_bus : j.at("buses"))
      {
        typename SystemModelData<RealT, IdxT>::BusDataT bus_data;
        raw_bus.get_to(bus_data);
        if (!raw_bus.contains("number"))
        {
          bus_data.bus_id = next_bus_id;
        }
        if (!bus_data.freq_base.has_value())
        {
          bus_data.freq_base = sm.freq_base;
        }
        bus_name_to_id[bus_data.name] = bus_data.bus_id;
        sm.bus.push_back(bus_data);
        ++next_bus_id;
      }

      for (const auto& raw_component : j.at("components"))
      {
        const auto kind = raw_component.at("class").template get<std::string>();

        if (kind == "BranchLumpedConstant")
        {
          typename SystemModelData<RealT, IdxT>::BranchLumpedConstantDataT component_data;
          parseNamedPortComponent<RealT, IdxT>(raw_component, component_data, bus_name_to_id);
          sm.branch_lumped_constant.push_back(component_data);
        }
        else if (kind == "LoadRL")
        {
          typename SystemModelData<RealT, IdxT>::LoadRLDataT component_data;
          parseNamedPortComponent<RealT, IdxT>(raw_component, component_data, bus_name_to_id);
          sm.load_rl.push_back(component_data);
        }
        else if (kind == "VoltageSource")
        {
          typename SystemModelData<RealT, IdxT>::VoltageSourceDataT component_data;
          parseNamedPortComponent<RealT, IdxT>(raw_component, component_data, bus_name_to_id);
          sm.voltage_source.push_back(component_data);
        }
        else if (kind == "Breaker")
        {
          typename SystemModelData<RealT, IdxT>::BreakerDataT component_data;
          parseNamedPortComponent<RealT, IdxT>(raw_component, component_data, bus_name_to_id);
          sm.breaker.push_back(component_data);
        }
        else
        {
          Log::error() << "\n\tInvalid EMT component class: \"" << kind << "\"." << std::endl;
        }
      }

      if (j.contains("monitors"))
      {
        for (const auto& raw_monitor : j.at("monitors"))
        {
          const auto file_name = raw_monitor.value("file_name", std::string{});
          const auto fmt_str   = raw_monitor.at("format").template get<std::string>();
          const auto format    = magic_enum::enum_cast<MonitorFormat>(fmt_str, magic_enum::case_insensitive);
          const auto delim     = raw_monitor.value("delim", std::string(","));
          if (format.has_value())
          {
            sm.monitor_sink.push_back({file_name, format.value(), delim});
          }
          else
          {
            Log::error() << "\n\tInvalid EMT monitor format: \"" << fmt_str << "\"." << std::endl;
          }
        }
      }
    }
  } // namespace EMT
} // namespace GridKit

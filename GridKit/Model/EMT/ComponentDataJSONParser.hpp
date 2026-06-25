#pragma once

#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>

#include <magic_enum/magic_enum.hpp>
#include <nlohmann/json.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename DataT>
    std::string componentContext(const DataT& data)
    {
      return data.device_class + " \"" + data.disambiguation_string + "\"";
    }

    template <typename DataT>
    void readComponentIdentity(const nlohmann::json& j,
                               DataT&                data,
                               const std::string&    expected_class)
    {
      j.at("class").get_to(data.device_class);
      j.at("id").get_to(data.disambiguation_string);

      if (data.device_class != expected_class)
      {
        throw std::invalid_argument(expected_class + ": invalid class \"" + data.device_class + "\"");
      }
    }

    template <typename Ports, typename IdxT>
      requires std::is_enum_v<Ports>
    void readPorts(const nlohmann::json&  ports_json,
                   std::map<Ports, IdxT>& ports,
                   const std::string&     context)
    {
      if (!ports_json.is_object())
      {
        throw std::invalid_argument(context + ": ports must be an object");
      }

      ports.clear();
      for (const auto& raw_port : ports_json.items())
      {
        auto key = magic_enum::enum_cast<Ports>(raw_port.key());
        if (!key.has_value())
        {
          throw std::invalid_argument(context + ": invalid port \"" + raw_port.key() + "\"");
        }

        raw_port.value().get_to(ports[key.value()]);
      }
    }

    template <typename DataT>
    void readComponentPorts(const nlohmann::json& j, DataT& data)
    {
      if (j.contains("ports"))
      {
        readPorts<typename DataT::Ports, typename DataT::IdxT>(
            j.at("ports"), data.ports, componentContext(data));
      }
    }

    template <typename DataT>
    void readComponentData(const nlohmann::json& j,
                           DataT&                data,
                           const std::string&    expected_class)
    {
      readComponentIdentity(j, data, expected_class);
      readComponentPorts(j, data);
    }

    template <typename Ports, typename IdxT>
    IdxT requiredPort(const std::map<Ports, IdxT>& ports,
                      Ports                        port,
                      const std::string&           context)
    {
      auto it = ports.find(port);
      if (it == ports.end())
      {
        std::stringstream ss;
        ss << context << ": missing required port \"" << magic_enum::enum_name(port) << "\"";
        throw std::invalid_argument(ss.str());
      }

      return it->second;
    }
  } // namespace EMT
} // namespace GridKit

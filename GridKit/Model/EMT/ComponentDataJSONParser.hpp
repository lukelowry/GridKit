#pragma once

#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string_view>

#include <magic_enum/magic_enum.hpp>
#include <nlohmann/json.hpp>

#include <GridKit/Model/EMT/ComponentData.hpp>
#include <GridKit/Utilities/Logger/Logger.hpp>

namespace GridKit
{
  namespace EMT
  {
    using json = nlohmann::json;
    using Log  = ::GridKit::Utilities::Logger;

    inline GridKit::Model::Events::PhaseMask parsePhaseMask(std::string_view token)
    {
      using PhaseMask = GridKit::Model::Events::PhaseMask;

      if (token == "none")
      {
        return PhaseMask::none();
      }

      std::uint8_t bits = 0u;
      for (char ch : token)
      {
        switch (ch)
        {
        case 'a':
        case 'A':
          if ((bits & 0b001u) != 0u)
          {
            throw std::invalid_argument("Duplicate EMT phase token");
          }
          bits |= 0b001u;
          break;
        case 'b':
        case 'B':
          if ((bits & 0b010u) != 0u)
          {
            throw std::invalid_argument("Duplicate EMT phase token");
          }
          bits |= 0b010u;
          break;
        case 'c':
        case 'C':
          if ((bits & 0b100u) != 0u)
          {
            throw std::invalid_argument("Duplicate EMT phase token");
          }
          bits |= 0b100u;
          break;
        default:
          throw std::invalid_argument("Invalid EMT phase token");
        }
      }

      return PhaseMask::fromBits(bits);
    }

    template <typename RealT>
    PhaseVector<RealT> parsePhaseVector(const json& value)
    {
      if (!value.is_array() || value.size() != 3)
      {
        throw std::invalid_argument("EMT phase vector must contain exactly three entries");
      }

      PhaseVector<RealT> parsed{};
      for (size_t i = 0; i < 3; ++i)
      {
        parsed[i] = value.at(i).template get<RealT>();
      }
      return parsed;
    }

    template <typename RealT>
    PhaseMatrix<RealT> parsePhaseMatrix(const json& value)
    {
      if (!value.is_array() || value.size() != 3)
      {
        throw std::invalid_argument("EMT phase matrix must contain exactly three rows");
      }

      PhaseMatrix<RealT> parsed{};
      for (size_t row = 0; row < 3; ++row)
      {
        parsed[row] = parsePhaseVector<RealT>(value.at(row));
      }
      return parsed;
    }

    template <typename RealT,
              typename IdxT,
              typename Parameters,
              typename Ports,
              typename MonitorableVariables>
      requires std::is_enum_v<Parameters>
               && std::is_enum_v<Ports>
               && std::is_enum_v<MonitorableVariables>
    typename ComponentData<RealT, IdxT, Parameters, Ports, MonitorableVariables>::ParameterValue
    parseParameterValue(const json& value)
    {
      using DataT = ComponentData<RealT, IdxT, Parameters, Ports, MonitorableVariables>;

      if (value.is_boolean())
      {
        return typename DataT::ParameterValue{value.template get<bool>()};
      }

      if (value.is_number())
      {
        return typename DataT::ParameterValue{value.template get<RealT>()};
      }

      if (value.is_string())
      {
        return typename DataT::ParameterValue{parsePhaseMask(value.template get<std::string>())};
      }

      if (value.is_array())
      {
        if (!value.empty() && value.at(0).is_array())
        {
          return typename DataT::ParameterValue{parsePhaseMatrix<RealT>(value)};
        }
        return typename DataT::ParameterValue{parsePhaseVector<RealT>(value)};
      }

      throw std::invalid_argument("Unsupported EMT component parameter value");
    }

    template <typename RealT,
              typename IdxT,
              typename Parameters,
              typename Ports,
              typename MonitorableVariables>
      requires std::is_enum_v<Parameters>
               && std::is_enum_v<Ports>
               && std::is_enum_v<MonitorableVariables>
    void parseComponentDataCommon(const json&                                                          j,
                                  ComponentData<RealT, IdxT, Parameters, Ports, MonitorableVariables>& c)
    {
      j.at("class").get_to(c.device_class);
      c.disambiguation_string = j.value("name", j.value("id", std::string{}));

      for (const auto& raw_parameter : j.at("params").items())
      {
        auto key = magic_enum::enum_cast<Parameters>(raw_parameter.key(), magic_enum::case_insensitive);
        if (key.has_value())
        {
          c.parameters[key.value()] =
              parseParameterValue<RealT, IdxT, Parameters, Ports, MonitorableVariables>(raw_parameter.value());
        }
        else
        {
          Log::error() << "\n\tInvalid EMT component parameter: \"" << raw_parameter.key() << "\"." << std::endl;
        }
      }

      if (j.contains("mon"))
      {
        for (const auto& raw_monitored_variable : j.at("mon"))
        {
          auto var_name  = raw_monitored_variable.template get<std::string>();
          auto monitored = magic_enum::enum_cast<MonitorableVariables>(var_name, magic_enum::case_insensitive);
          if (monitored.has_value())
          {
            c.monitored_variables.insert(monitored.value());
          }
          else
          {
            Log::error() << "\n\tInvalid EMT monitored variable: \"" << var_name << "\"." << std::endl;
          }
        }
      }
    }

    template <typename RealT,
              typename IdxT,
              typename Parameters,
              typename Ports,
              typename MonitorableVariables>
      requires std::is_enum_v<Parameters>
               && std::is_enum_v<Ports>
               && std::is_enum_v<MonitorableVariables>
    void from_json(const json& j, ComponentData<RealT, IdxT, Parameters, Ports, MonitorableVariables>& c)
    {
      parseComponentDataCommon(j, c);

      for (const auto& raw_port : j.at("ports").items())
      {
        auto key = magic_enum::enum_cast<Ports>(raw_port.key(), magic_enum::case_insensitive);
        if (key.has_value())
        {
          raw_port.value().get_to(c.ports[key.value()]);
        }
        else
        {
          Log::error() << "\n\tInvalid EMT component port: \"" << raw_port.key() << "\"." << std::endl;
        }
      }
    }
  } // namespace EMT
} // namespace GridKit

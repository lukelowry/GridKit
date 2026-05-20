#pragma once

#include <map>
#include <set>
#include <string>
#include <type_traits>
#include <variant>

#include <GridKit/Model/EMT/Math/PhaseMath.hpp>
#include <GridKit/Model/Events.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <typename RealT,
              typename IdxT,
              typename Parameters,
              typename Ports,
              typename MonitorableVariables>
      requires std::is_enum_v<Parameters>
               && std::is_enum_v<Ports>
               && std::is_enum_v<MonitorableVariables>
    struct ComponentData
    {
      using ParameterValue = std::variant<bool,
                                          RealT,
                                          IdxT,
                                          PhaseVector<RealT>,
                                          PhaseMatrix<RealT>,
                                          GridKit::Model::Events::PhaseMask>;

      std::string                          device_class;
      std::map<Parameters, ParameterValue> parameters;
      std::map<Ports, IdxT>                ports;
      std::set<MonitorableVariables>       monitored_variables;
      std::string                          disambiguation_string;

    protected:
      ComponentData() = default;
    };
  } // namespace EMT
} // namespace GridKit

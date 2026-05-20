#pragma once

#include <optional>
#include <set>
#include <string>

namespace GridKit
{
  namespace EMT
  {
    enum class BusMonitorableVariables
    {
      va,
      vb,
      vc,
      dva,
      dvb,
      dvc,
      ifa,
      ifb,
      ifc
    };

    template <typename RealT, typename IdxT>
    struct BusData
    {
      std::string name;

      RealT vm{0.0};
      RealT va{0.0};

      IdxT bus_id{0};

      std::optional<RealT> freq_base;

      using MonitorableVariables = BusMonitorableVariables;

      std::set<MonitorableVariables> monitored_variables;
    };
  } // namespace EMT
} // namespace GridKit

#pragma once

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
      vc
    };

    template <typename real_type, typename index_type>
    struct BusData
    {
      using RealT                = real_type;
      using IdxT                 = index_type;
      using MonitorableVariables = BusMonitorableVariables;

      std::string name;
      IdxT        bus_id{0};

      std::set<MonitorableVariables> monitored_variables;
    };
  } // namespace EMT
} // namespace GridKit

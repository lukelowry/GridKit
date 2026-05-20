#pragma once

#include <GridKit/Model/EMT/ComponentData.hpp>

namespace GridKit
{
  namespace EMT
  {
    enum class VoltageSourceParameters
    {
      e,
      phi,
      r,
      frequency,
      omega0
    };

    enum class VoltageSourcePorts
    {
      bus
    };

    enum class VoltageSourceMonitorableVariables
    {
      ia,
      ib,
      ic
    };

    template <typename RealT, typename IdxT>
    struct VoltageSourceData : public ComponentData<RealT,
                                                    IdxT,
                                                    VoltageSourceParameters,
                                                    VoltageSourcePorts,
                                                    VoltageSourceMonitorableVariables>
    {
      VoltageSourceData() = default;

      using Parameters           = VoltageSourceParameters;
      using Ports                = VoltageSourcePorts;
      using MonitorableVariables = VoltageSourceMonitorableVariables;
    };
  } // namespace EMT
} // namespace GridKit

#pragma once

#include <GridKit/Model/EMT/ComponentData.hpp>

namespace GridKit
{
  namespace EMT
  {
    enum class LoadRLParameters
    {
      r,
      l
    };

    enum class LoadRLPorts
    {
      ac
    };

    enum class LoadRLMonitorableVariables
    {
      ia,
      ib,
      ic,
      dia,
      dib,
      dic
    };

    template <typename RealT, typename IdxT>
    struct LoadRLData : public ComponentData<RealT,
                                             IdxT,
                                             LoadRLParameters,
                                             LoadRLPorts,
                                             LoadRLMonitorableVariables>
    {
      LoadRLData() = default;

      using Parameters           = LoadRLParameters;
      using Ports                = LoadRLPorts;
      using MonitorableVariables = LoadRLMonitorableVariables;
    };
  } // namespace EMT
} // namespace GridKit

#pragma once

#include <GridKit/Model/EMT/ComponentData.hpp>
#include <GridKit/Model/Events.hpp>

namespace GridKit
{
  namespace EMT
  {
    enum class BreakerParameters
    {
      closed
    };

    enum class BreakerPorts
    {
      from,
      to
    };

    enum class BreakerMonitorableVariables
    {
      ia,
      ib,
      ic,
      dia,
      dib,
      dic
    };

    template <typename RealT, typename IdxT>
    struct BreakerData : public ComponentData<RealT,
                                              IdxT,
                                              BreakerParameters,
                                              BreakerPorts,
                                              BreakerMonitorableVariables>
    {
      BreakerData()
      {
        this->parameters[Parameters::closed] = GridKit::Model::Events::PhaseMask::abc();
      }

      using Parameters           = BreakerParameters;
      using Ports                = BreakerPorts;
      using MonitorableVariables = BreakerMonitorableVariables;
    };
  } // namespace EMT
} // namespace GridKit

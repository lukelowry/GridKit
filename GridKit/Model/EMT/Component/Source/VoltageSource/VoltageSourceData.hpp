#pragma once

#include <GridKit/Model/EMT/ComponentData.hpp>

namespace GridKit
{
  namespace EMT
  {
    enum class VoltageSourceParameters
    {
      E,
      phi,
      omega,
      Rs,
      Ls
    };

    enum class VoltageSourceBuses : size_t
    {
      bus,
      SIZE
    };

    enum class VoltageSourceSignalInputs : size_t
    {
      SIZE
    };

    enum class VoltageSourceSignalOutputs : size_t
    {
      SIZE
    };

    enum class VoltageSourceMonitorableVariables
    {
      ea,
      eb,
      ec,
      ia,
      ib,
      ic
    };

    template <typename real_type, typename index_type>
    struct VoltageSourceData : public ComponentData<real_type,
                                                    index_type,
                                                    VoltageSourceParameters,
                                                    VoltageSourceBuses,
                                                    VoltageSourceSignalInputs,
                                                    VoltageSourceSignalOutputs,
                                                    VoltageSourceMonitorableVariables>
    {
      using Parameters           = VoltageSourceParameters;
      using Buses                = VoltageSourceBuses;
      using SignalInputs         = VoltageSourceSignalInputs;
      using SignalOutputs        = VoltageSourceSignalOutputs;
      using MonitorableVariables = VoltageSourceMonitorableVariables;
    };
  } // namespace EMT
} // namespace GridKit

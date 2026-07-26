#pragma once

#include <GridKit/Model/EMT/ComponentData.hpp>

namespace GridKit
{
  namespace EMT
  {
    enum class SwitchParameters
    {
      N
    };

    enum class SwitchBuses : size_t
    {
      bus1,
      bus2,
      SIZE
    };

    enum class SwitchSignalInputs : size_t
    {
      open,
      SIZE
    };

    enum class SwitchSignalOutputs : size_t
    {
      SIZE
    };

    enum class SwitchMonitorableVariables
    {
      open,
      i12a,
      i12b,
      i12c
    };

    template <typename real_type, typename index_type>
    struct SwitchData : public ComponentData<real_type,
                                             index_type,
                                             SwitchParameters,
                                             SwitchBuses,
                                             SwitchSignalInputs,
                                             SwitchSignalOutputs,
                                             SwitchMonitorableVariables>
    {
      using Parameters           = SwitchParameters;
      using Buses                = SwitchBuses;
      using SignalInputs         = SwitchSignalInputs;
      using SignalOutputs        = SwitchSignalOutputs;
      using MonitorableVariables = SwitchMonitorableVariables;
    };
  } // namespace EMT
} // namespace GridKit

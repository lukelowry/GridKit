#pragma once

#include <GridKit/Model/EMT/ComponentData.hpp>

namespace GridKit
{
  namespace EMT
  {
    enum class LineLumpedParameters
    {
      dx,
      Rp,
      Lp,
      Gp,
      Cp
    };

    enum class LineLumpedBuses : size_t
    {
      bus1,
      bus2,
      SIZE
    };

    enum class LineLumpedSignalInputs : size_t
    {
      SIZE
    };

    enum class LineLumpedSignalOutputs : size_t
    {
      SIZE
    };

    enum class LineLumpedMonitorableVariables
    {
      i12a,
      i12b,
      i12c,
      i_sh1a,
      i_sh1b,
      i_sh1c,
      i_sh2a,
      i_sh2b,
      i_sh2c
    };

    template <typename real_type, typename index_type>
    struct LineLumpedData : public ComponentData<real_type,
                                                 index_type,
                                                 LineLumpedParameters,
                                                 LineLumpedBuses,
                                                 LineLumpedSignalInputs,
                                                 LineLumpedSignalOutputs,
                                                 LineLumpedMonitorableVariables>
    {
      using Parameters           = LineLumpedParameters;
      using Buses                = LineLumpedBuses;
      using SignalInputs         = LineLumpedSignalInputs;
      using SignalOutputs        = LineLumpedSignalOutputs;
      using MonitorableVariables = LineLumpedMonitorableVariables;
    };
  } // namespace EMT
} // namespace GridKit

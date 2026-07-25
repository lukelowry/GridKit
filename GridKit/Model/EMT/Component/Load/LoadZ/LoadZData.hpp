#pragma once

#include <GridKit/Model/EMT/ComponentData.hpp>

namespace GridKit
{
  namespace EMT
  {
    enum class LoadZParameters
    {
      R,
      L
    };

    enum class LoadZBuses : size_t
    {
      bus,
      SIZE
    };

    enum class LoadZSignalInputs : size_t
    {
      enable,
      SIZE
    };

    enum class LoadZSignalOutputs : size_t
    {
      SIZE
    };

    enum class LoadZMonitorableVariables
    {
      ia,
      ib,
      ic
    };

    template <typename real_type, typename index_type>
    struct LoadZData : public ComponentData<real_type,
                                            index_type,
                                            LoadZParameters,
                                            LoadZBuses,
                                            LoadZSignalInputs,
                                            LoadZSignalOutputs,
                                            LoadZMonitorableVariables>
    {
      using Parameters           = LoadZParameters;
      using Buses                = LoadZBuses;
      using SignalInputs         = LoadZSignalInputs;
      using SignalOutputs        = LoadZSignalOutputs;
      using MonitorableVariables = LoadZMonitorableVariables;
    };
  } // namespace EMT
} // namespace GridKit

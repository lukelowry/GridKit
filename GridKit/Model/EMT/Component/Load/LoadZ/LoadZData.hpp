#pragma once

#include <map>

#include <GridKit/Model/EMT/ComponentData.hpp>
#include <GridKit/Model/EMT/Operators/Rational/VectorFit/VectorFitData.hpp>

namespace GridKit
{
  namespace EMT
  {
    enum class LoadZParameters
    {
      N
    };

    enum class LoadZSubmodels : size_t
    {
      Z,
      SIZE
    };

    enum class LoadZBuses : size_t
    {
      bus,
      SIZE
    };

    enum class LoadZSignalInputs : size_t
    {
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
      using Submodels            = LoadZSubmodels;
      using Buses                = LoadZBuses;
      using SignalInputs         = LoadZSignalInputs;
      using SignalOutputs        = LoadZSignalOutputs;
      using MonitorableVariables = LoadZMonitorableVariables;

      /// Mapping of submodels to their rational-operator parameters
      std::map<Submodels, VectorFitData<real_type, index_type>> submodels;
    };
  } // namespace EMT
} // namespace GridKit

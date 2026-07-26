#pragma once

#include <map>

#include <GridKit/Model/EMT/ComponentData.hpp>
#include <GridKit/Model/EMT/Operators/Rational/VectorFit/VectorFitData.hpp>

namespace GridKit
{
  namespace EMT
  {
    enum class LineLumpedParameters
    {
      N,
      K,
      conductors,
      dx
    };

    enum class LineLumpedSubmodels : size_t
    {
      Zp,
      Yp,
      SIZE
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
      using Submodels            = LineLumpedSubmodels;
      using Buses                = LineLumpedBuses;
      using SignalInputs         = LineLumpedSignalInputs;
      using SignalOutputs        = LineLumpedSignalOutputs;
      using MonitorableVariables = LineLumpedMonitorableVariables;

      /// Mapping of submodels to their rational-operator parameters
      std::map<Submodels, VectorFitData<real_type, index_type>> submodels;
    };
  } // namespace EMT
} // namespace GridKit

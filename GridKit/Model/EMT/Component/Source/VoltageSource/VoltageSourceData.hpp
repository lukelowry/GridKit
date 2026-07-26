#pragma once

#include <map>

#include <GridKit/Model/EMT/ComponentData.hpp>
#include <GridKit/Model/EMT/Operators/Rational/VectorFit/VectorFitData.hpp>

namespace GridKit
{
  namespace EMT
  {
    enum class VoltageSourceParameters
    {
      N,
      E,
      phi,
      omega
    };

    enum class VoltageSourceSubmodels : size_t
    {
      Z,
      SIZE
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
      using Submodels            = VoltageSourceSubmodels;
      using Buses                = VoltageSourceBuses;
      using SignalInputs         = VoltageSourceSignalInputs;
      using SignalOutputs        = VoltageSourceSignalOutputs;
      using MonitorableVariables = VoltageSourceMonitorableVariables;

      /// Mapping of submodels to their rational-operator parameters
      std::map<Submodels, VectorFitData<real_type, index_type>> submodels;
    };
  } // namespace EMT
} // namespace GridKit

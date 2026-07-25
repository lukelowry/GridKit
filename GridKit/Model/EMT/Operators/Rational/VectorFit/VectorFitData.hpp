#pragma once

#include <GridKit/Model/EMT/ComponentData.hpp>

namespace GridKit
{
  namespace EMT
  {
    enum class VectorFitParameters
    {
      D,
      E,
      poles,
      residues
    };

    enum class VectorFitBuses : size_t
    {
      SIZE
    };

    enum class VectorFitSignalInputs : size_t
    {
      input_a,
      input_b,
      input_c,
      SIZE
    };

    enum class VectorFitSignalOutputs : size_t
    {
      out_a,
      out_b,
      out_c,
      SIZE
    };

    template <typename real_type, typename index_type>
    struct VectorFitData
      : public ComponentData<real_type,
                             index_type,
                             VectorFitParameters,
                             VectorFitBuses,
                             VectorFitSignalInputs,
                             VectorFitSignalOutputs,
                             std::monostate>
    {
      using Parameters           = VectorFitParameters;
      using Buses                = VectorFitBuses;
      using SignalInputs         = VectorFitSignalInputs;
      using SignalOutputs        = VectorFitSignalOutputs;
      using MonitorableVariables = std::monostate;
    };
  } // namespace EMT
} // namespace GridKit

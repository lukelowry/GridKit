#pragma once

#include <GridKit/Model/EMT/ComponentData.hpp>

namespace GridKit
{
  namespace EMT
  {
    enum class BranchLumpedConstantParameters
    {
      r,
      l,
      g,
      c,
      length
    };

    enum class BranchLumpedConstantPorts
    {
      from,
      to
    };

    enum class BranchLumpedConstantMonitorableVariables
    {
      ia,
      ib,
      ic,
      dia,
      dib,
      dic
    };

    template <typename RealT, typename IdxT>
    struct BranchLumpedConstantData
      : public ComponentData<RealT,
                             IdxT,
                             BranchLumpedConstantParameters,
                             BranchLumpedConstantPorts,
                             BranchLumpedConstantMonitorableVariables>
    {
      BranchLumpedConstantData() = default;

      using Parameters           = BranchLumpedConstantParameters;
      using Ports                = BranchLumpedConstantPorts;
      using MonitorableVariables = BranchLumpedConstantMonitorableVariables;
    };
  } // namespace EMT
} // namespace GridKit

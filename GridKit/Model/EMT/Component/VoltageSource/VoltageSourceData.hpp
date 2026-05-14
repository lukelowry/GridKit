#pragma once

#include <GridKit/Model/EMT/PhaseMath.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    struct VoltageSourceData
    {
      PhaseVector<RealT> e{RealT{0.0}, RealT{0.0}, RealT{0.0}};
      PhaseVector<RealT> phi{RealT{0.0}, RealT{0.0}, RealT{0.0}};
      PhaseVector<RealT> r{RealT{0.0}, RealT{0.0}, RealT{0.0}};
      RealT              omega0{0.0};
    };
  } // namespace EMT
} // namespace GridKit

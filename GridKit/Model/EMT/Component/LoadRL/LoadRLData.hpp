#pragma once

#include <GridKit/Model/EMT/PhaseMath.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    struct LoadRLData
    {
      PhaseVector<RealT> r{RealT{0.0}, RealT{0.0}, RealT{0.0}};
      PhaseVector<RealT> l{RealT{0.0}, RealT{0.0}, RealT{0.0}};
    };
  } // namespace EMT
} // namespace GridKit

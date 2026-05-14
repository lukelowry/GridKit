#pragma once

#include <GridKit/Model/Events.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    struct BreakerData
    {
      GridKit::Model::Events::PhaseMask closed{GridKit::Model::Events::PhaseMask::abc()};
    };
  } // namespace EMT
} // namespace GridKit

#pragma once

#include <GridKit/Model/Events.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    struct BusFaultData
    {
      GridKit::Model::Events::PhaseMask active{GridKit::Model::Events::PhaseMask::none()};
      RealT                             r{RealT{1.0}};
      RealT                             minimum_resistance{RealT{1.0e-6}};
    };
  } // namespace EMT
} // namespace GridKit

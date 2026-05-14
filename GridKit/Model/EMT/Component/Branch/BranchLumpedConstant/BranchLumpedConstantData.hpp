#pragma once

#include <GridKit/Model/EMT/PhaseMath.hpp>

namespace GridKit
{
  namespace EMT
  {
    template <class RealT, typename IdxT>
    struct BranchLumpedConstantData
    {
      PhaseMatrix<RealT> r{};      // ohm / m
      PhaseMatrix<RealT> l{};      // H / m
      PhaseMatrix<RealT> g{};      // S / m
      PhaseMatrix<RealT> c{};      // F / m
      RealT              length{}; // m
    };
  } // namespace EMT
} // namespace GridKit

/**
 * @file BusData.hpp
 * @brief Data for EMT abc buses.
 */

#pragma once

#include <array>

namespace GridKit
{
  namespace EMT
  {
    template <typename RealT, typename IdxT>
    struct BusData
    {
      std::array<RealT, 3> v0{1.0, 0.0, 0.0};  ///< Initial abc voltages
      std::array<RealT, 3> vp0{0.0, 0.0, 0.0}; ///< Initial abc voltage derivatives
    };

  } // namespace EMT
} // namespace GridKit

/**
 * @file VoltageSourceData.hpp
 * @brief Data for ideal balanced EMT abc voltage sources.
 */

#pragma once

#include <array>

namespace GridKit
{
  namespace EMT
  {
    template <typename RealT, typename IdxT>
    struct VoltageSourceData
    {
      RealT                amplitude{1.0};  ///< Phase peak voltage
      RealT                frequency{60.0}; ///< Frequency in Hz
      RealT                phase{0.0};      ///< Phase-a angle at t = 0, radians
      std::array<RealT, 3> current0{};      ///< Initial source current
    };

  } // namespace EMT
} // namespace GridKit

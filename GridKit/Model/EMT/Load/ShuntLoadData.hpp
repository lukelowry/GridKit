/**
 * @file ShuntLoadData.hpp
 * @brief Data for EMT abc shunt loads.
 */

#pragma once

#include <array>

namespace GridKit
{
  namespace EMT
  {
    template <typename RealT, typename IdxT>
    struct ShuntLoadData
    {
      std::array<RealT, 9> conductance{}; ///< Row-major abc conductance matrix
      bool                 closed{true};  ///< Initial load switch state
    };

  } // namespace EMT
} // namespace GridKit

/**
 * @file DelayData.hpp
 *
 * @brief Static scalar delay-operator data.
 *
 */

#pragma once

namespace GridKit
{
  namespace EMT
  {
    namespace Operators
    {
      namespace Shift
      {
        template <typename scalar_type, typename index_type>
        struct DelayData
        {
          using ScalarT = scalar_type;
          using IdxT    = index_type;

          ScalarT delay{0.0};
          ScalarT dt_min{0.0};
        };
      } // namespace Shift
    } // namespace Operators
  } // namespace EMT
} // namespace GridKit

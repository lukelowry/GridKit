/**
 * @file PropagationData.hpp
 *
 * @brief Static data for the composite EMT propagation operator.
 *
 */

#pragma once

#include <vector>

#include <GridKit/Model/EMT/Operators/Rational/StateSpace/StateSpaceData.hpp>

namespace GridKit
{
  namespace EMT
  {
    namespace Operators
    {
      namespace Shift
      {
        template <typename scalar_type, typename index_type>
        struct PropagationData
        {
          using ScalarT        = scalar_type;
          using IdxT           = index_type;
          using StateSpaceData = Rational::StateSpaceData<ScalarT, IdxT>;

          StateSpaceData       input;
          std::vector<ScalarT> tau;
          ScalarT              dt_min{0.0};
          StateSpaceData       output;
        };
      } // namespace Shift
    } // namespace Operators
  } // namespace EMT
} // namespace GridKit

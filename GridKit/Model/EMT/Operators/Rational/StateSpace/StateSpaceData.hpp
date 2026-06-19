/**
 * @file StateSpaceData.hpp
 *
 * @brief Static fitted rational state-space operator data.
 *
 */

#pragma once

#include <complex>
#include <vector>

namespace GridKit
{
  namespace EMT
  {
    namespace Operators
    {
      namespace Rational
      {
        template <typename scalar_type, typename index_type>
        struct StateSpaceData
        {
          using ScalarT  = scalar_type;
          using IdxT     = index_type;
          using ComplexT = std::complex<ScalarT>;

          IdxT N{0};
          IdxT K{0};
          IdxT Q{0};

          std::vector<ScalarT>  D;     // N x K, row-major
          std::vector<ScalarT>  E;     // N x K, row-major
          std::vector<ComplexT> poles; // Q
          std::vector<ComplexT> C;     // N x Q, row-major
          std::vector<ComplexT> B;     // Q x K, row-major
        };
      } // namespace Rational
    } // namespace Operators
  } // namespace EMT
} // namespace GridKit

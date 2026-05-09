/**
 * @file RationalApproxData.hpp
 * @brief Data for real-valued rational matrix approximations.
 */

#pragma once

#include <cstddef>
#include <vector>

namespace GridKit
{
  namespace EMT
  {
    /**
     * @brief Coefficients for a square vector-fitted matrix rational approximation.
     *
     * Matrix entries use row-major storage. Per-mode vector entries use
     * mode-major storage, i.e. `entry[mode * dimension + component]`.
     */
    template <typename RealT, typename IdxT>
    struct RationalApproxData
    {
      size_t dimension{0}; ///< Input and output vector dimension

      std::vector<RealT> d; ///< Direct-feedthrough matrix, row-major dimension x dimension
      std::vector<RealT> e; ///< Input-derivative matrix, row-major dimension x dimension

      std::vector<RealT> p; ///< Real poles
      std::vector<RealT> b; ///< Per-real-pole input coupling vectors, mode-major
      std::vector<RealT> c; ///< Per-real-pole output residue vectors, mode-major

      std::vector<RealT> complex_p_real; ///< Real parts of stored positive-imaginary complex-pair poles
      std::vector<RealT> complex_p_imag; ///< Positive imaginary parts of stored complex-pair poles

      std::vector<RealT> complex_b_real; ///< Real parts of per-pair input coupling vectors, mode-major
      std::vector<RealT> complex_b_imag; ///< Imaginary parts of per-pair input coupling vectors, mode-major
      std::vector<RealT> complex_c_real; ///< Real parts of per-pair output residue vectors, mode-major
      std::vector<RealT> complex_c_imag; ///< Imaginary parts of per-pair output residue vectors, mode-major
    };

  } // namespace EMT
} // namespace GridKit

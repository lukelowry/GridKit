/**
 * @file ConvolutionVecData.hpp
 * @brief Modeling data for the vector-valued vector-fitting convolution helper
 */

#pragma once

#include <cstddef>
#include <vector>

#include <GridKit/Model/PhasorDynamics/ComponentData.hpp>

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Convolution
    {
      /// Placeholder scalar parameter enum for compatibility with ComponentData.
      enum class ConvolutionVecParameters
      {
        NONE
      };

      /// Placeholder port enum for compatibility with ComponentData.
      enum class ConvolutionVecPorts
      {
        input,
        output
      };

      /// Placeholder monitor enum for compatibility with ComponentData.
      enum class ConvolutionVecMonitorableVariables
      {
        NONE
      };

      /**
       * @brief Contains modeling data for a square MIMO vector-fitting convolution helper.
       *
       * Matrix entries use row-major storage. Per-mode vector entries use
       * mode-major storage, i.e. `entry[mode * dimension + component]`.
       */
      template <typename RealT, typename IdxT>
      struct ConvolutionVecData : public ComponentData<RealT,
                                                       IdxT,
                                                       ConvolutionVecParameters,
                                                       ConvolutionVecPorts,
                                                       ConvolutionVecMonitorableVariables>
      {
        ConvolutionVecData() = default;

        using Parameters           = ConvolutionVecParameters;
        using Ports                = ConvolutionVecPorts;
        using MonitorableVariables = ConvolutionVecMonitorableVariables;

        size_t dimension{0}; ///< Input and output vector dimension

        std::vector<RealT> d; ///< Direct-feedthrough matrix, row-major dimension x dimension
        std::vector<RealT> e; ///< Input-derivative matrix, row-major dimension x dimension

        std::vector<RealT> p; ///< Real vector-fitting poles
        std::vector<RealT> b; ///< Per-pole input coupling vectors, mode-major
        std::vector<RealT> c; ///< Per-pole output residue vectors, mode-major

        std::vector<RealT> complex_p_real; ///< Real parts of stored positive-imaginary complex-pair poles
        std::vector<RealT> complex_p_imag; ///< Positive imaginary parts of stored complex-pair poles

        std::vector<RealT> complex_b_real; ///< Real parts of per-pair input coupling vectors, mode-major
        std::vector<RealT> complex_b_imag; ///< Imaginary parts of per-pair input coupling vectors, mode-major
        std::vector<RealT> complex_c_real; ///< Real parts of per-pair output residue vectors, mode-major
        std::vector<RealT> complex_c_imag; ///< Imaginary parts of per-pair output residue vectors, mode-major

        std::vector<RealT> u0;  ///< Initial input vector
        std::vector<RealT> up0; ///< Initial input derivative vector
      };

    } // namespace Convolution
  } // namespace PhasorDynamics
} // namespace GridKit

/**
 * @file ConvolutionVFData.hpp
 * @brief Modeling data for the vector-fitting convolution helper
 */

#pragma once

#include <vector>

#include <GridKit/Model/PhasorDynamics/ComponentData.hpp>

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Convolution
    {
      /// Scalar parameters for a vector-fitting convolution helper
      enum class ConvolutionVFParameters
      {
        d,
        e,
        u0,
        up0
      };

      /// Signal ports for a vector-fitting convolution helper
      enum class ConvolutionVFPorts
      {
        input,
        output
      };

      /// Placeholder enum for monitor compatibility
      enum class ConvolutionVFMonitorableVariables
      {
        NONE
      };

      /**
       * @brief Contains modeling data for a vector-fitting convolution helper
       *
       * Coefficient arrays are stored as typed fields because they do not fit
       * the scalar `ComponentData::parameters` map used by the current parser.
       */
      template <typename RealT, typename IdxT>
      struct ConvolutionVFData : public ComponentData<RealT,
                                                      IdxT,
                                                      ConvolutionVFParameters,
                                                      ConvolutionVFPorts,
                                                      ConvolutionVFMonitorableVariables>
      {
        ConvolutionVFData() = default;

        using Parameters           = ConvolutionVFParameters;
        using Ports                = ConvolutionVFPorts;
        using MonitorableVariables = ConvolutionVFMonitorableVariables;

        RealT d{0.0};   ///< Direct-feedthrough coefficient
        RealT e{0.0};   ///< Input-derivative coefficient
        RealT u0{0.0};  ///< Initial input value
        RealT up0{0.0}; ///< Initial input derivative

        std::vector<RealT> p; ///< Vector-fitting poles
        std::vector<RealT> r; ///< Vector-fitting residues
      };

    } // namespace Convolution
  } // namespace PhasorDynamics
} // namespace GridKit

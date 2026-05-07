/**
 * @file ConvolutionVFDependencyTracking.cpp
 */

#include "ConvolutionVFImpl.hpp"

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Convolution
    {
      /**
       * @brief Jacobian evaluation not implemented yet
       *
       * @tparam ScalarT - Scalar data type
       * @tparam IdxT    - Index data type
       * @return int - error code, 0 = success
       */
      template <class ScalarT, typename IdxT>
      int ConvolutionVF<ScalarT, IdxT>::evaluateJacobian()
      {
        Log::misc() << "Evaluate Jacobian for ConvolutionVF..." << std::endl;
        Log::misc() << "Jacobian evaluation not implemented!" << std::endl;
        return 0;
      }

      // Available template instantiations
      template class ConvolutionVF<DependencyTracking::Variable, long int>;
      template class ConvolutionVF<DependencyTracking::Variable, size_t>;
    } // namespace Convolution
  } // namespace PhasorDynamics
} // namespace GridKit

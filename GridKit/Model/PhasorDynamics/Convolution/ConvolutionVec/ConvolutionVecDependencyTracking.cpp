/**
 * @file ConvolutionVecDependencyTracking.cpp
 */

#include "ConvolutionVecImpl.hpp"

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Convolution
    {
      /**
       * @brief Jacobian evaluation not implemented for DependencyTracking scalar type
       */
      template <class ScalarT, typename IdxT>
      int ConvolutionVec<ScalarT, IdxT>::evaluateJacobian()
      {
        Log::misc() << "Evaluate Jacobian for ConvolutionVec..." << std::endl;
        Log::misc() << "Jacobian evaluation not implemented!" << std::endl;
        return 0;
      }

      // Available template instantiations
      template class ConvolutionVec<DependencyTracking::Variable, long int>;
      template class ConvolutionVec<DependencyTracking::Variable, size_t>;
    } // namespace Convolution
  } // namespace PhasorDynamics
} // namespace GridKit

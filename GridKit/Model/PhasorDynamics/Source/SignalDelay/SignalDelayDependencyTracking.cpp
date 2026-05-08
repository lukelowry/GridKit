/**
 * @file SignalDelayDependencyTracking.cpp
 */

#include "SignalDelayImpl.hpp"

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Source
    {
      /**
       * @brief Jacobian evaluation not implemented yet
       */
      template <class ScalarT, typename IdxT>
      int SignalDelay<ScalarT, IdxT>::evaluateJacobian()
      {
        Log::misc() << "Evaluate Jacobian for SignalDelay..." << std::endl;
        Log::misc() << "Jacobian evaluation not implemented!" << std::endl;
        return 0;
      }

      // Available template instantiations
      template class SignalDelay<DependencyTracking::Variable, long int>;
      template class SignalDelay<DependencyTracking::Variable, size_t>;
    } // namespace Source
  } // namespace PhasorDynamics
} // namespace GridKit

/**
 * @file CsvSignalSourceDependencyTracking.cpp
 */

#include "CsvSignalSourceImpl.hpp"

namespace GridKit
{
  namespace PhasorDynamics
  {
    namespace Source
    {
      /**
       * @brief Jacobian evaluation not implemented for DependencyTracking scalar type
       */
      template <class ScalarT, typename IdxT>
      int CsvSignalSource<ScalarT, IdxT>::evaluateJacobian()
      {
        Log::misc() << "Evaluate Jacobian for CsvSignalSource..." << std::endl;
        Log::misc() << "Jacobian evaluation not implemented!" << std::endl;
        return 0;
      }

      // Available template instantiations
      template class CsvSignalSource<DependencyTracking::Variable, long int>;
      template class CsvSignalSource<DependencyTracking::Variable, size_t>;
    } // namespace Source
  } // namespace PhasorDynamics
} // namespace GridKit

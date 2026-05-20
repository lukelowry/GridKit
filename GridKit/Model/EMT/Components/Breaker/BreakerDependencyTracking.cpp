#include "BreakerImpl.hpp"

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    int Breaker<ScalarT, IdxT>::evaluateJacobian()
    {
      return 0;
    }

    template class Breaker<DependencyTracking::Variable, long int>;
    template class Breaker<DependencyTracking::Variable, size_t>;
  } // namespace EMT
} // namespace GridKit

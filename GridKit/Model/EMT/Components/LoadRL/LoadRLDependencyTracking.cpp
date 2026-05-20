#include "LoadRLImpl.hpp"

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    int LoadRL<ScalarT, IdxT>::evaluateJacobian()
    {
      return 0;
    }

    template class LoadRL<DependencyTracking::Variable, long int>;
    template class LoadRL<DependencyTracking::Variable, size_t>;
  } // namespace EMT
} // namespace GridKit

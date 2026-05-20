#include "BranchLumpedConstantImpl.hpp"

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    int BranchLumpedConstant<ScalarT, IdxT>::evaluateJacobian()
    {
      return 0;
    }

    template class BranchLumpedConstant<DependencyTracking::Variable, long int>;
    template class BranchLumpedConstant<DependencyTracking::Variable, size_t>;
  } // namespace EMT
} // namespace GridKit

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

    template class BranchLumpedConstant<double, long int>;
    template class BranchLumpedConstant<double, size_t>;
  } // namespace EMT
} // namespace GridKit

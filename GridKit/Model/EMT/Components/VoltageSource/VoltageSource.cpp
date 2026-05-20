#include "VoltageSourceImpl.hpp"

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    int VoltageSource<ScalarT, IdxT>::evaluateJacobian()
    {
      return 0;
    }

    template class VoltageSource<double, long int>;
    template class VoltageSource<double, size_t>;
  } // namespace EMT
} // namespace GridKit

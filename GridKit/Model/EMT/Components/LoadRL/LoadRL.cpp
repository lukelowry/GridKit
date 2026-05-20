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

    template class LoadRL<double, long int>;
    template class LoadRL<double, size_t>;
  } // namespace EMT
} // namespace GridKit

#include "BusImpl.hpp"

namespace GridKit
{
  namespace EMT
  {
    template <class ScalarT, typename IdxT>
    int Bus<ScalarT, IdxT>::evaluateJacobian()
    {
      return 0;
    }

    template class Bus<double, long int>;
    template class Bus<double, size_t>;
  } // namespace EMT
} // namespace GridKit

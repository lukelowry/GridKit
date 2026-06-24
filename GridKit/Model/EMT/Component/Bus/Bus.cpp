#include "BusImpl.hpp"

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    int Bus<scalar_type, index_type>::evaluateJacobian()
    {
      return 0;
    }

    template class Bus<double, long int>;
    template class Bus<double, size_t>;
  } // namespace EMT
} // namespace GridKit

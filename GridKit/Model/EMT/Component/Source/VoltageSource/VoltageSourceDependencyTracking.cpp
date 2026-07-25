#include "VoltageSourceImpl.hpp"

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    int VoltageSource<scalar_type, index_type>::evaluateJacobian()
    {
      return 0;
    }

    template class VoltageSource<DependencyTracking::Variable, long int>;
    template class VoltageSource<DependencyTracking::Variable, size_t>;
  } // namespace EMT
} // namespace GridKit

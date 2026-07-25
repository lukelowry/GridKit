#include "LoadZImpl.hpp"

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    int LoadZ<scalar_type, index_type>::evaluateJacobian()
    {
      return 0;
    }

    template class LoadZ<DependencyTracking::Variable, long int>;
    template class LoadZ<DependencyTracking::Variable, size_t>;
  } // namespace EMT
} // namespace GridKit

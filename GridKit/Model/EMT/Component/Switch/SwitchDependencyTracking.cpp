#include "SwitchImpl.hpp"

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    int Switch<scalar_type, index_type>::evaluateJacobian()
    {
      return 0;
    }

    template class Switch<DependencyTracking::Variable, long int>;
    template class Switch<DependencyTracking::Variable, size_t>;
  } // namespace EMT
} // namespace GridKit

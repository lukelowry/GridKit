#include "LineLumpedImpl.hpp"

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    int LineLumped<scalar_type, index_type>::evaluateJacobian()
    {
      return 0;
    }

    template class LineLumped<DependencyTracking::Variable, long int>;
    template class LineLumped<DependencyTracking::Variable, size_t>;
  } // namespace EMT
} // namespace GridKit

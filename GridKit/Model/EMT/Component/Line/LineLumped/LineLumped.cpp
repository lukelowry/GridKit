#include "LineLumpedImpl.hpp"

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    int LineLumped<scalar_type, index_type>::evaluateJacobian()
    {
      Log::error() << "EMT::LineLumped: Jacobian requires an Enzyme-enabled build\n";
      return 1;
    }

    template class LineLumped<double, long int>;
    template class LineLumped<double, size_t>;
  } // namespace EMT
} // namespace GridKit

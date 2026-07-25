#include "LoadZImpl.hpp"

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    int LoadZ<scalar_type, index_type>::evaluateJacobian()
    {
      Log::error() << "EMT::LoadZ: Jacobian requires an Enzyme-enabled build\n";
      return 1;
    }

    template class LoadZ<double, long int>;
    template class LoadZ<double, size_t>;
  } // namespace EMT
} // namespace GridKit

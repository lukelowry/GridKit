#include "VoltageSourceImpl.hpp"

namespace GridKit
{
  namespace EMT
  {
    template <typename scalar_type, typename index_type>
    int VoltageSource<scalar_type, index_type>::evaluateJacobian()
    {
      Log::error() << "EMT::VoltageSource: Jacobian requires an Enzyme-enabled build\n";
      return 1;
    }

    template class VoltageSource<double, long int>;
    template class VoltageSource<double, size_t>;
  } // namespace EMT
} // namespace GridKit

// VectorFit has runtime pole count, so its Jacobian is stamped analytically.
#include "VectorFitImpl.hpp"

namespace GridKit
{
  namespace EMT
  {
    template class VectorFit<double, long int>;
    template class VectorFit<double, size_t>;
  } // namespace EMT
} // namespace GridKit

#include "SystemModelImpl.hpp"

namespace GridKit::EMT
{
  template <typename scalar_type, typename index_type>
  bool SystemModel<scalar_type, index_type>::hasJacobian()
  {
    return false;
  }

  template class SystemModel<DependencyTracking::Variable, std::size_t>;
} // namespace GridKit::EMT

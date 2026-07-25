#include "SystemModelImpl.hpp"

namespace GridKit::EMT
{
  template <typename scalar_type, typename index_type>
  bool SystemModel<scalar_type, index_type>::hasJacobian()
  {
    Log::warning() << "GridKit was not built with Enzyme; EMT will use a "
                      "dense solver Jacobian.\n";
    return false;
  }

  template class SystemModel<double, std::size_t>;
} // namespace GridKit::EMT

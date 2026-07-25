#include "SystemModelImpl.hpp"

namespace GridKit::EMT
{
  template <typename scalar_type, typename index_type>
  bool SystemModel<scalar_type, index_type>::hasJacobian()
  {
    bool available = true;
    for (auto* bus : buses_)
    {
      available = available && bus->hasJacobian();
    }
    for (auto* component : components_)
    {
      available = available && component->hasJacobian();
    }
    return available;
  }

  template class SystemModel<double, std::size_t>;
} // namespace GridKit::EMT

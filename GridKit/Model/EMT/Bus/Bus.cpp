/**
 * @file Bus.cpp
 */

#include <GridKit/Model/EMT/Bus/BusImpl.hpp>

namespace GridKit
{
  namespace EMT
  {
    template class Bus<double, long int>;
    template class Bus<double, size_t>;
  } // namespace EMT
} // namespace GridKit

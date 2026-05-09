/**
 * @file VoltageSource.cpp
 */

#include <GridKit/Model/EMT/Source/VoltageSource/VoltageSourceImpl.hpp>

namespace GridKit
{
  namespace EMT
  {
    template class VoltageSource<double, long int>;
    template class VoltageSource<double, size_t>;
  } // namespace EMT
} // namespace GridKit

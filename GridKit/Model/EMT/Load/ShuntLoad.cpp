/**
 * @file ShuntLoad.cpp
 */

#include <GridKit/Model/EMT/Load/ShuntLoadImpl.hpp>

namespace GridKit
{
  namespace EMT
  {
    template class ShuntLoad<double, long int>;
    template class ShuntLoad<double, size_t>;
  } // namespace EMT
} // namespace GridKit

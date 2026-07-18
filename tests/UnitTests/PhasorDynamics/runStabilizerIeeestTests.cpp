#include "StabilizerIeeestTests.hpp"

int main()
{
  GridKit::Testing::TestingResults result;

  GridKit::Testing::StabilizerIeeestTests<double, size_t> test;

  result += test.constructor();

  result += test.init<0>();
  result += test.init<1>();
  result += test.init<2>();
  result += test.init<3>();
  result += test.init<4>();

  result += test.zeroInitialResidual<0>();
  result += test.zeroInitialResidual<1>();
  result += test.zeroInitialResidual<2>();
  result += test.zeroInitialResidual<3>();
  result += test.zeroInitialResidual<4>();

  result += test.residual<0>();
  result += test.residual<1>();
  result += test.residual<2>();
  result += test.residual<3>();
  result += test.residual<4>();

  result += test.tags<0>();
  result += test.tags<1>();
  result += test.tags<2>();
  result += test.tags<3>();
  result += test.tags<4>();

  result += test.verify();
  result += test.factory();
  result += test.symmetricNotch();

#ifdef GRIDKIT_ENABLE_ENZYME
  result += test.jacobian<0>();
  result += test.jacobian<1>();
  result += test.jacobian<2>();
  result += test.jacobian<3>();
  result += test.jacobian<4>();
#endif

  return result.summary();
}

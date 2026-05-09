#include "SystemModelTests.hpp"

int main()
{
  GridKit::Testing::TestingResults result;

  GridKit::Testing::SystemModelTests<double, size_t> test;

  result += test.ownershipAndAllocation();
  result += test.initialization();
  result += test.residualDispatch();
  result += test.tagDifferentiable();
  result += test.jacobianAssembly();

  return result.summary();
}

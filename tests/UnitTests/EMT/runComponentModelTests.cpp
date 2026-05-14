#include "ComponentModelTests.hpp"

int main()
{
  GridKit::Testing::TestingResults                         result;
  GridKit::Testing::EMTComponentModelTests<double, size_t> test;

  result += test.busInitialization();
  result += test.loadRLInitialization();
  result += test.voltageSourceResidual();
  result += test.branchValidation();
  result += test.branchInitialization();
  result += test.branchResidual();
  result += test.discoveredStructure();
  result += test.zeroStateSparsity();
  result += test.enzymeJacobian();

  return result.summary();
}

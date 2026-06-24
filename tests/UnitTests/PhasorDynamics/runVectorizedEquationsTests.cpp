#include "VectorizedEquationsTests.hpp"

int main()
{
  GridKit::Testing::TestingResults                           result;
  GridKit::Testing::VectorizedEquationsTests<double, size_t> test;

  result += test.blockAssignment();
  result += test.sliceAssignment();
  result += test.residualLayout();
  result += test.signalSumResidualLayout();

#ifdef GRIDKIT_ENABLE_ENZYME
  result += test.enzymeJacobian();
  result += test.busDerivativeInternalJacobian();
  result += test.busDerivativeBusJacobian();
  result += test.busDerivativeOffDiagonalJacobian();
  result += test.signalSumEnzymeJacobian();
#endif

  return result.summary();
}

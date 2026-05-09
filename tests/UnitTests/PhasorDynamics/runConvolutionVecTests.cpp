#include "ConvolutionVecTests.hpp"

int main()
{
  GridKit::Testing::TestingResults result;

  GridKit::Testing::ConvolutionVecTests<double, size_t> test;

  result += test.constructor();
  result += test.verifyFailures();
  result += test.zeroInitialResidual();
  result += test.complexZeroInitialResidual();
  result += test.residual();
  result += test.complexResidual();
  result += test.jacobian();
  result += test.complexJacobian();
  result += test.frequencyResponse();

  return result.summary();
}

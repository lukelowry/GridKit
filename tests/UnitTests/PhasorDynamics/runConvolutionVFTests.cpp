#include "ConvolutionVFTests.hpp"

int main()
{
  GridKit::Testing::TestingResults result;

  GridKit::Testing::ConvolutionVFTests<double, size_t> test;

  result += test.constructor();
  result += test.zeroInitialResidual();
  result += test.residual();
  result += test.jacobian();

  return result.summary();
}

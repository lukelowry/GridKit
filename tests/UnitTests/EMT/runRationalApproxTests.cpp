#include "RationalApproxTests.hpp"

int main()
{
  GridKit::Testing::TestingResults result;

  GridKit::Testing::RationalApproxTests<double, size_t> test;

  result += test.constructor();
  result += test.verifyFailures();
  result += test.initialization();
  result += test.outputAndResidual();
  result += test.jacobianEntries();
  result += test.frequencyResponse();

  return result.summary();
}

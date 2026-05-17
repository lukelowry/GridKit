#include "FrequencyDependentBranchTests.hpp"

int main()
{
  GridKit::Testing::TestingResults                                  result;
  GridKit::Testing::EMTFrequencyDependentBranchTests<double, size_t> test;

  result += test.constantDResidual();
  result += test.zeroFitResidual();
  result += test.monitorCurrents();

  return result.summary();
}

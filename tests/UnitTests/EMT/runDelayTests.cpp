#include "DelayTests.hpp"

int main()
{
  GridKit::Testing::TestingResults                result;
  GridKit::Testing::EMTDelayTests<double, size_t> test;

  result += test.constructorValidation();
  result += test.derivedParameters();
  result += test.initializationUsesInputHistory();
  result += test.customInitializationSetsEachSection();
  result += test.residual();
  result += test.jacobian();

  return result.summary();
}

#include "LineLumpedTests.hpp"

int main()
{
  using namespace GridKit;
  using namespace GridKit::Testing;

  GridKit::Testing::TestingResults                     result;
  GridKit::Testing::EMTLineLumpedTests<double, size_t> test;

  result += test.allocationAndTags();
  result += test.parserConstantParameters();
  result += test.parserFittedParameters();
  result += test.parserRejectsInvalidParameters();
  result += test.verifyRejectsInvalidConfiguration();
  result += test.initializeConstantOperators();
  result += test.residualAndTerminalCurrents();
  result += test.systemModel();
#ifdef GRIDKIT_ENABLE_ENZYME
  result += test.enzymeJacobian();
#endif

  return result.summary();
}

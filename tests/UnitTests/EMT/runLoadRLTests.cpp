#include "LoadRLTests.hpp"

int main()
{
  using namespace GridKit;
  using namespace GridKit::Testing;

  GridKit::Testing::TestingResults                 result;
  GridKit::Testing::EMTLoadRLTests<double, size_t> test;

  result += test.parser();
  result += test.parserRejectsInvalidParameters();
  result += test.verifyRejectsInvalidConfiguration();
  result += test.initializeFromPhasor();
  result += test.residualAndBusInjection();
  result += test.tagsAndTolerance();
  result += test.systemModel();
  result += test.sourceLoadSystemModel();
#ifdef GRIDKIT_ENABLE_ENZYME
  result += test.enzymeJacobian();
#endif

  return result.summary();
}

#include "VoltageSourceTests.hpp"

int main()
{
  using namespace GridKit;
  using namespace GridKit::Testing;

  GridKit::Testing::TestingResults                        result;
  GridKit::Testing::EMTVoltageSourceTests<double, size_t> test;

  result += test.parser();
  result += test.parserRejectsInvalidParameters();
  result += test.verifyRejectsInvalidConfiguration();
  result += test.residualAndTime();
  result += test.systemModel();
#ifdef GRIDKIT_ENABLE_ENZYME
  result += test.enzymeJacobian();
#endif

  return result.summary();
}

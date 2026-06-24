#include "VectorFitTests.hpp"

int main()
{
  using namespace GridKit;
  using namespace GridKit::Testing;

  GridKit::Testing::TestingResults                    result;
  GridKit::Testing::EMTVectorFitTests<double, size_t> test;

  result += test.allocationAndSignals();
  result += test.q0Residual();
  result += test.poleResidualLayout();
  result += test.initializeAffineInput();
  result += test.differentiabilityTags();
  result += test.parser();
  result += test.verifyRejectsInvalidSignals();
#ifdef GRIDKIT_ENABLE_ENZYME
  result += test.enzymeJacobian();
#endif

  return result.summary();
}

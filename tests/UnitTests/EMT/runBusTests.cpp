#include "BusTests.hpp"

int main()
{
  using namespace GridKit;
  using namespace GridKit::Testing;

  GridKit::Testing::TestingResults           result;
  GridKit::Testing::BusTests<double, size_t> test;

  result += test.defaultConstruction();
  result += test.vectorConstruction();
  result += test.residual();
  result += test.unsupportedPhasorAliases();
  result += test.systemModel();
  result += test.cooPointerAxpyThreePhase();
#ifdef GRIDKIT_ENABLE_ENZYME
  result += test.jacobian();
#endif

  return result.summary();
}

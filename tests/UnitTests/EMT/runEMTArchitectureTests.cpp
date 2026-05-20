#include "EMTArchitectureTests.hpp"

int main()
{
  using namespace GridKit;
  using namespace GridKit::Testing;

  TestingResults                       result;
  EMTArchitectureTests<double, size_t> test;

  result += test.busLifecycle();
  result += test.componentLifecycle();
  result += test.manualSystemModel();
  result += test.rationalApproxSkeleton();

  return result.summary();
}

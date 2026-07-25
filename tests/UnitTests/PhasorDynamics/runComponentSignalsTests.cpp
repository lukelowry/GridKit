#include "ComponentSignalsTests.hpp"

int main()
{
  using namespace GridKit::Testing;

  TestingResults                        result;
  ComponentSignalsTests<double, size_t> test;

  result += test.signalNodeDerivative();
  result += test.componentSignalsDerivative();
  result += test.constantSignalSourceDerivative();

  return result.summary();
}

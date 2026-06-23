#include "ComponentSignalsTests.hpp"

int main()
{
  GridKit::Testing::TestingResults                        result;
  GridKit::Testing::ComponentSignalsTests<double, size_t> test;

  result += test.scalarCompatibility();
  result += test.vectorSizeOne();
  result += test.vectorSizeThree();
  result += test.vectorSizeFive();
  result += test.invalidPorts();
  result += test.componentUsage();

  return result.summary();
}

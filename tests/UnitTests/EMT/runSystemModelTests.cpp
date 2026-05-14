#include "SystemModelTests.hpp"

int main()
{
  GridKit::Testing::TestingResults                      result;
  GridKit::Testing::EMTSystemModelTests<double, size_t> test;

  result += test.systemModelData();
  result += test.layout();
  result += test.residual();
  result += test.threeTerminalComponent();
  result += test.terminalWiring();
  result += test.ports();
  result += test.jacobian();

  return result.summary();
}

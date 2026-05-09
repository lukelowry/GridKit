#include "VoltageSourceTests.hpp"

int main()
{
  GridKit::Testing::TestingResults result;

  GridKit::Testing::VoltageSourceTests<double, size_t> test;

  result += test.initialization();
  result += test.residual();
  result += test.jacobian();

  return result.summary();
}

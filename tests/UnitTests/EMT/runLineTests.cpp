#include "LineTests.hpp"

int main()
{
  GridKit::Testing::TestingResults result;

  GridKit::Testing::LineTests<double, size_t> test;

  result += test.constructor();
  result += test.initialization();
  result += test.residual();
  result += test.tagDifferentiable();
  result += test.jacobian();

  return result.summary();
}

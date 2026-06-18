#include "IdaTests.hpp"

int main()
{
  using namespace GridKit;
  using namespace GridKit::Testing;

  GridKit::Testing::TestingResults           result;
  GridKit::Testing::IdaTests<double, size_t> test;

  result += test.test();
  result += test.algebraic_error_control();
  result += test.log_evaluator_algebraic();
  result += test.log_evaluator_derivative_scaling();
  result += test.log_evaluator_alpha_scaling();

  return result.summary();
}

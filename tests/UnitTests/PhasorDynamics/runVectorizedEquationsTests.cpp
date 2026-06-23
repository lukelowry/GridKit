#include "VectorizedEquationsTests.hpp"

int main()
{
  GridKit::Testing::TestingResults                           result;
  GridKit::Testing::VectorizedEquationsTests<double, size_t> test;

  result += test.blockAssignment();
  result += test.sliceAssignment();
  result += test.residualLayout();

#ifdef GRIDKIT_ENABLE_ENZYME
  result += test.enzymeJacobian();
#endif

  return result.summary();
}

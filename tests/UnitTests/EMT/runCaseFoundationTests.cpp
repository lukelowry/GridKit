#include <cstddef>

#include "CaseFoundationTests.hpp"

int main()
{
  GridKit::Testing::TestingResults                              result;
  GridKit::Testing::EMTCaseFoundationTests<double, std::size_t> test;

  result += test.jsonSupport();
  result += test.phaseMaskReader();
  result += test.paramReader();
  result += test.descriptorCoverage();
  result += test.monitorResolution();
  result += test.constructorRethrowing();

  return result.summary();
}

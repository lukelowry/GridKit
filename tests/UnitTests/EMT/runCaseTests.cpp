#include <cstddef>

#include "CaseTests.hpp"

int main()
{
  GridKit::Testing::TestingResults                    result;
  GridKit::Testing::EMTCaseTests<double, std::size_t> test;

  result += test.happyPath();
  result += test.signalPortWiring();
  result += test.pathLoading();
  result += test.errorCases();

  return result.summary();
}

#include <cstddef>

#include "SolverFileTests.hpp"

int main()
{
  GridKit::Testing::TestingResults                          result;
  GridKit::Testing::EMTSolverFileTests<double, std::size_t> test;

  result += test.parserHappyPath();
  result += test.parserDefaults();
  result += test.parserErrors();
  result += test.scheduleAndOutput();

  return result.summary();
}

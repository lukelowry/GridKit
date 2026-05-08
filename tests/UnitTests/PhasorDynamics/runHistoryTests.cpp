#include "HistoryTests.hpp"

int main()
{
  GridKit::Testing::TestingResults result;
  GridKit::Testing::HistoryTests   test;

  result += test.scalarLookup();
  result += test.pruningAndRestart();

  return result.summary();
}

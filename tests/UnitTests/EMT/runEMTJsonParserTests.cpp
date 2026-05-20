#include "EMTJsonParserTests.hpp"

int main()
{
  using namespace GridKit;
  using namespace GridKit::Testing;

  TestingResults     result;
  EMTJsonParserTests test;

  result += test.parseTwoBusCase();

  return result.summary();
}

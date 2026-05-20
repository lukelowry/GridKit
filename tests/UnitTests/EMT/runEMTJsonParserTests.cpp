#include "EMTJsonParserTests.hpp"

int main()
{
  using namespace GridKit;
  using namespace GridKit::Testing;

  TestingResults     result;
  EMTJsonParserTests test;

  result += test.parseTwoBusFixture();

  return result.summary();
}

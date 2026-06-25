#include "SystemModelTests.hpp"

int main()
{
  using namespace GridKit;
  using namespace GridKit::Testing;

  GridKit::Testing::TestingResults                      result;
  GridKit::Testing::EMTSystemModelTests<double, size_t> test;

  result += test.parser();
  result += test.construction();
  result += test.connectivityErrors();

  return result.summary();
}

#include "ShuntLoadTests.hpp"

int main()
{
  GridKit::Testing::TestingResults result;

  GridKit::Testing::ShuntLoadTests<double, size_t> test;

  result += test.switching();

  return result.summary();
}

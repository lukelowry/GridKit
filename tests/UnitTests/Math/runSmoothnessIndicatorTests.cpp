#include "SmoothnessIndicatorTests.hpp"

int main()
{
  GridKit::Testing::TestingResults result;

  GridKit::Testing::SmoothnessIndicatorTests<double> test;

  result += test.clamp();
  result += test.slew();
  result += test.rampsat();
  result += test.ramp();
  result += test.antiWindupIndicator();

  return result.summary();
}

#include "FitFileTests.hpp"

int main()
{
  GridKit::Testing::TestingResults             result;
  GridKit::Testing::EMTFitFileTests<double>    test;

  result += test.validCharacteristicAdmittance();
  result += test.rejectsInvalidFiles();
  result += test.pathAndHashValidation();

  return result.summary();
}

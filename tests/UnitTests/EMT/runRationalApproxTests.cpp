#include "RationalApproxTests.hpp"

int main()
{
  GridKit::Testing::TestingResults                 result;
  GridKit::Testing::EMTRationalApproxTests<double> test;

  result += test.countsAndValidation();
  result += test.directAndDerivativeOutput();
  result += test.realPoleResidual();
  result += test.complexPairResidual();
  result += test.fullMatrixResidueOutput();
  result += test.initialization();
  result += test.independentApplications();

  return result.summary();
}

#include "EMTComponentModelTests.hpp"
#include "EMTLocalMapTests.hpp"
#include "EMTSparseADTests.hpp"
#include "EMTSystemModelTests.hpp"

int main()
{
  using namespace GridKit;
  using namespace GridKit::Testing;

  TestingResults result;

  EMTLocalMapTests local_map_tests;
  result += local_map_tests.gatherScatterAndAccumulate();
  result += local_map_tests.csrDeduplicatesStableSlots();

  EMTSparseADTests sparse_ad_tests;
  result += sparse_ad_tests.affinePatternTracksYAndYpSeparately();
  result += sparse_ad_tests.nonlinearValuePathMatchesFiniteDifference();

  EMTComponentModelTests<double, size_t> component_tests;
  result += component_tests.loadRLValidationResidualInitAndSparse();
  result += component_tests.voltageSourceValidationResidualAndSparse();
  result += component_tests.branchValidationResidualInitAndSparse();

  EMTSystemModelTests<double, size_t> system_tests;
  result += system_tests.twoBusSystemResidualJacobianAndTags();

  return result.summary();
}

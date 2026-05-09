#include "SystemModelDataTests.hpp"

int main()
{
  GridKit::Testing::TestingResults       result;
  GridKit::Testing::SystemModelDataTests test;

  result += test.parsesEmtCase();
  result += test.buildsSystemModelFromData();
  result += test.rejectsInvalidAdmittance();
  result += test.rejectsUnresolvedReferences();

  return result.summary();
}

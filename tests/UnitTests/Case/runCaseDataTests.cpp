#include "CaseDataTests.hpp"

int main()
{
  GridKit::Testing::TestingResults result;
  GridKit::Testing::CaseDataTests  test;

  result += test.parsesMinimalCase();
  result += test.requiresRootArrays();
  result += test.rejectsUnknownFields();
  result += test.rejectsDuplicateIds();

  return result.summary();
}

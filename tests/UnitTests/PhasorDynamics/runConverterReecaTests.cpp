#include "ConverterReecaTests.hpp"

int main()
{
  GridKit::Testing::TestingResults result;

  GridKit::Testing::ConverterReecaTests<double, size_t> test;

  result += test.constructionAndValidation();
  result += test.parameterValidation();
  result += test.reecaSignalsInitializationAndResidual();
  result += test.rejectsHalfConnectedElectricalFeedback();
  result += test.reecaCommandSignalInitialization();
  result += test.reecaElectricalFeedbackUsesMvaBase();
  result += test.reecaReferenceFallbackAtAngle();
  result += test.zeroTimeConstantTags();
  result += test.outputAvailability();
  result += test.priorityInitialization();
  result += test.initializesSaturatedStartConsistently();
#ifdef GRIDKIT_ENABLE_ENZYME
  result += test.jacobian();
#endif
  result += test.jsonParseAndSystemAssembly();

  return result.summary();
}

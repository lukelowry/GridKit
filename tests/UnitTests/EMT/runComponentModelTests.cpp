#include "ComponentModelTests.hpp"

int main()
{
  GridKit::Testing::TestingResults                         result;
  GridKit::Testing::EMTComponentModelTests<double, size_t> test;

  result += test.busInitialization();
  result += test.eventPhaseMask();
  result += test.loadRLInitialization();
  result += test.voltageSourceResidual();
  result += test.branchValidation();
  result += test.branchInitialization();
  result += test.branchResidual();
  result += test.breakerResidual();
  result += test.breakerEvents();
  result += test.breakerMonitorCsv();
  result += test.busFaultEvents();
  result += test.busFaultMonitorCsv();
  result += test.breakerJacobian();
  result += test.discoveredStructure();
  result += test.zeroStateSparsity();
  result += test.enzymeJacobian();
  result += test.callbackVariableMonitor();
  result += test.emtMonitorCsv();
  result += test.emtMonitorValidation();

  return result.summary();
}

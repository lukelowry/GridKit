add_executable(test_emt_three_bus_validator EMTThreeBusValidator.cpp)
add_dependencies(EMTDynamicSimulation test_emt_three_bus_validator)

set(_emt_three_bus_directory ${CMAKE_BINARY_DIR}/examples/EMT/ThreeBus)
set(_emt_three_bus_solver IEEE13ReducedABC.solver.json)
set(_emt_three_bus_output emt_three_bus_fault.csv)

add_test(
  NAME EMTThreeBusSimulationTest
  COMMAND $<TARGET_FILE:EMTDynamicSimulation> ${_emt_three_bus_solver}
  WORKING_DIRECTORY ${_emt_three_bus_directory})
set_tests_properties(
  EMTThreeBusSimulationTest
  PROPERTIES FIXTURES_SETUP
             EMTThreeBusSimulation
             TIMEOUT
             120)

add_test(
  NAME EMTThreeBusValidationTest
  COMMAND $<TARGET_FILE:test_emt_three_bus_validator> ${_emt_three_bus_output}
  WORKING_DIRECTORY ${_emt_three_bus_directory})
set_tests_properties(
  EMTThreeBusValidationTest
  PROPERTIES FIXTURES_REQUIRED
             EMTThreeBusSimulation
             TIMEOUT
             30)

install(TARGETS test_emt_three_bus_validator)

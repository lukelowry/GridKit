add_executable(test_emt_three_bus_validator EMTThreeBusValidator.cpp)
add_dependencies(EMTDynamicSimulation test_emt_three_bus_validator)

set(_emt_three_bus_directory ${CMAKE_BINARY_DIR}/examples/EMT/ThreeBus)
set(_emt_three_bus_solver ThreeBus.solver.json)
set(_emt_three_bus_output ThreeBus.csv)
set(_emt_three_bus_adaptive_directory ${_emt_three_bus_directory}/Adaptive)
set(_emt_three_bus_adaptive_solver ThreeBusAdaptive.solver.json)
set(_emt_three_bus_adaptive_output ${_emt_three_bus_adaptive_directory}/ThreeBusAdaptive.csv)
set(_emt_three_bus_fixed_directory ${_emt_three_bus_directory}/Fixed)
set(_emt_three_bus_fixed_solver ThreeBusFixed.solver.json)
set(_emt_three_bus_fixed_output ${_emt_three_bus_fixed_directory}/ThreeBusFixed.csv)

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

add_test(
  NAME EMTThreeBusShortAdaptiveSimulationTest
  COMMAND $<TARGET_FILE:EMTDynamicSimulation> ${_emt_three_bus_adaptive_solver}
  WORKING_DIRECTORY ${_emt_three_bus_adaptive_directory})
set_tests_properties(
  EMTThreeBusShortAdaptiveSimulationTest
  PROPERTIES FIXTURES_SETUP
             EMTThreeBusShortAdaptiveSimulation
             TIMEOUT
             120)

add_test(
  NAME EMTThreeBusShortFixedSimulationTest
  COMMAND $<TARGET_FILE:EMTDynamicSimulation> ${_emt_three_bus_fixed_solver}
  WORKING_DIRECTORY ${_emt_three_bus_fixed_directory})
set_tests_properties(
  EMTThreeBusShortFixedSimulationTest
  PROPERTIES FIXTURES_SETUP
             EMTThreeBusShortFixedSimulation
             TIMEOUT
             120)

add_test(
  NAME EMTThreeBusIntegratorComparisonTest
  COMMAND
    $<TARGET_FILE:test_emt_three_bus_validator> --compare ${_emt_three_bus_adaptive_output}
    ${_emt_three_bus_fixed_output}
  WORKING_DIRECTORY ${_emt_three_bus_directory})
set_tests_properties(
  EMTThreeBusIntegratorComparisonTest
  PROPERTIES FIXTURES_REQUIRED
             "EMTThreeBusShortAdaptiveSimulation;EMTThreeBusShortFixedSimulation"
             TIMEOUT
             30)

function(
  add_emt_example_simulation_test
  _name
  _directory
  _solver)
  add_test(
    NAME ${_name}
    COMMAND $<TARGET_FILE:EMTDynamicSimulation> ${_solver}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/examples/EMT/${_directory})
  set_tests_properties(${_name} PROPERTIES TIMEOUT 120)
endfunction()

add_emt_example_simulation_test(EMTIEEE13SimulationTest IEEE13 IEEE13.solver.json)
set_tests_properties(
  EMTIEEE13SimulationTest
  PROPERTIES FIXTURES_SETUP
             EMTIEEE13Simulation
             TIMEOUT
             300)
add_test(
  NAME EMTIEEE13PeriodicStateValidationTest
  COMMAND $<TARGET_FILE:test_emt_three_bus_validator> --periodic IEEE13.csv
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/examples/EMT/IEEE13)
set_tests_properties(
  EMTIEEE13PeriodicStateValidationTest
  PROPERTIES FIXTURES_REQUIRED
             EMTIEEE13Simulation
             TIMEOUT
             30)
add_emt_example_simulation_test(EMTBalancedSteadyStateSimulationTest
                                StandardStudies/BalancedSteadyState BalancedSteadyState.solver.json)
add_emt_example_simulation_test(EMTBalancedThreePhaseFaultSimulationTest
                                StandardStudies/ThreePhaseFault ThreePhaseFault.solver.json)
add_emt_example_simulation_test(
  EMTBalancedSingleLineGroundFaultSimulationTest StandardStudies/SingleLineGroundFault
  SingleLineGroundFault.solver.json)
add_emt_example_simulation_test(EMTBalancedLoadRejectionSimulationTest
                                StandardStudies/LoadRejection LoadRejection.solver.json)

install(TARGETS test_emt_three_bus_validator)

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

add_emt_example_simulation_test(EMTIEEE13SimulationTest IEEE13 IEEE13FixedABC.solver.json)
add_emt_example_simulation_test(EMTBalancedSteadyStateSimulationTest StandardStudies
                                BalancedRLC3Bus.steady.solver.json)
add_emt_example_simulation_test(EMTBalancedThreePhaseFaultSimulationTest StandardStudies
                                BalancedRLC3Bus.fault.solver.json)
add_emt_example_simulation_test(EMTBalancedSingleLineGroundFaultSimulationTest StandardStudies
                                BalancedRLC3Bus.slg_fault.solver.json)
add_emt_example_simulation_test(EMTBalancedLoadRejectionSimulationTest StandardStudies
                                BalancedRLC3Bus.load_rejection.solver.json)

install(TARGETS test_emt_three_bus_validator)

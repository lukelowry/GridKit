gridkit_example_add_file(texas.case.json)
gridkit_example_add_file(texas.ref.csv)
gridkit_example_add_file(texas.solver.json)

add_test(
  NAME texas_pdsim
  COMMAND DynamicSimulation texas.solver.json
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

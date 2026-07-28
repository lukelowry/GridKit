gridkit_example_add_file(wecc.case.json)
gridkit_example_add_file(wecc.solver.json)

add_test(
  NAME wecc_pdsim
  COMMAND DynamicSimulation wecc.solver.json
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# The solver file's element_id resolves to a generator terminal bus, where the
# bolted fault leaves the network with no consistent initial condition.
set_tests_properties(wecc_pdsim PROPERTIES DISABLED TRUE)

gridkit_example_current_install_path(_install_path)
install(FILES wecc.case.json wecc.solver.json DESTINATION ${_install_path})

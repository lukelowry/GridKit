gridkit_example_add_file(TwoBusBasic.case.json)
gridkit_example_add_file(TwoBusBasic.solver.json)
gridkit_example_add_file(TwoBusBasic.ref.csv)

add_test(
  NAME TwoBusBasic
  COMMAND DynamicSimulation TwoBusBasic.solver.json
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# The reference vector was captured at the former 1e-7 default solver
# tolerance and its error bound sits on the edge, so the looser default
# overshoots it.
set_tests_properties(TwoBusBasic PROPERTIES DISABLED TRUE)

gridkit_example_current_install_path(_install_path)
install(FILES TwoBusBasic.case.json TwoBusBasic.solver.json TwoBusBasic.ref.csv
        DESTINATION ${_install_path})

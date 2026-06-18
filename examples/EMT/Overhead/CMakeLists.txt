gridkit_example_current_install_path(_install_path)

gridkit_example_add_file(overhead.line.json)
gridkit_example_add_file(overhead.solver.json)

add_test(NAME FrequencyResponseOverhead COMMAND FrequencyResponse overhead.solver.json)

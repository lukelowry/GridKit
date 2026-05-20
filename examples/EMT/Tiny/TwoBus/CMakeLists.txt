gridkit_example_current_install_path(_install_path)

add_custom_target(EMTTinyTwoBus DEPENDS EMTSim)

gridkit_example_add_file(TwoBus.case.json)
gridkit_example_add_file(TwoBus.solver.json)
gridkit_example_add_file(README.md)

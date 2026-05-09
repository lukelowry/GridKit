gridkit_example_current_install_path(_install_path)

add_executable(ThreeBusVoltageSourceJson ThreeBusVoltageSourceJson.cpp)
target_link_libraries(ThreeBusVoltageSourceJson
  GridKit::emt_components
  GridKit::solvers_dyn)
install(TARGETS ThreeBusVoltageSourceJson RUNTIME DESTINATION ${_install_path})

gridkit_example_add_file(ThreeBusVoltageSource.case.json)
gridkit_example_add_file(ThreeBusVoltageSource.solver.json)

add_test(NAME ThreeBusVoltageSourceJson
  COMMAND ThreeBusVoltageSourceJson ${CMAKE_CURRENT_BINARY_DIR}/ThreeBusVoltageSource.solver.json
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
add_test(NAME ThreeBusVoltageSourceJson_no_arg
  COMMAND ThreeBusVoltageSourceJson
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

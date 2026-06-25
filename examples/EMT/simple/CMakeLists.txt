gridkit_example_add_file(simple.case.json)
gridkit_example_add_file(simple.solver.json)
gridkit_example_add_file(simple.plot.py)

add_test(
  NAME EMTSimpleDynamicSimulation
  COMMAND $<TARGET_FILE:emt_dynamic_simulation> simple.solver.json
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
  add_test(
    NAME EMTSimplePlot
    COMMAND ${Python3_EXECUTABLE} simple.plot.py simple.csv simple.png
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  set_tests_properties(EMTSimplePlot PROPERTIES DEPENDS EMTSimpleDynamicSimulation)
endif()

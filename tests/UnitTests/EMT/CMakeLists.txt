add_executable(test_emt_system_model runSystemModelTests.cpp)
target_link_libraries(test_emt_system_model
  GridKit::emt
  GridKit::testing)

add_test(NAME EMTSystemModelTest COMMAND test_emt_system_model)

install(TARGETS test_emt_system_model
  RUNTIME DESTINATION bin)

add_executable(test_emt_component_models runComponentModelTests.cpp)
target_link_libraries(test_emt_component_models
  GridKit::emt
  GridKit::testing)

add_test(NAME EMTComponentModelTest COMMAND test_emt_component_models)

install(TARGETS test_emt_component_models
  RUNTIME DESTINATION bin)

add_executable(test_emt_case_foundation runCaseFoundationTests.cpp)
target_link_libraries(test_emt_case_foundation
  GridKit::emt_io
  GridKit::testing)

add_test(NAME EMTCaseFoundationTest COMMAND test_emt_case_foundation)

install(TARGETS test_emt_case_foundation
  RUNTIME DESTINATION bin)

add_executable(test_emt_case runCaseTests.cpp)
target_link_libraries(test_emt_case
  GridKit::emt_io
  GridKit::testing)

add_test(NAME EMTCaseTest COMMAND test_emt_case)

install(TARGETS test_emt_case
  RUNTIME DESTINATION bin)

add_executable(test_emt_solver_file runSolverFileTests.cpp)
target_link_libraries(test_emt_solver_file
  GridKit::emt_io
  GridKit::testing)

add_test(NAME EMTSolverFileTest COMMAND test_emt_solver_file)

install(TARGETS test_emt_solver_file
  RUNTIME DESTINATION bin)

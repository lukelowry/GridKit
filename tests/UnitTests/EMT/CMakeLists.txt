add_executable(test_emt_system_model runSystemModelTests.cpp)
target_link_libraries(test_emt_system_model
  GridKit::emt
  GridKit::testing)

add_test(NAME EMTSystemModelTest COMMAND test_emt_system_model)

install(TARGETS test_emt_system_model
  RUNTIME DESTINATION bin)

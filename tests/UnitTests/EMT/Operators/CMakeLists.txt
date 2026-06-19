add_executable(test_emt_delay runDelayTests.cpp)
target_link_libraries(
  test_emt_delay GridKit::emt_delay GridKit::testing)

add_test(NAME EMTDelayTest COMMAND $<TARGET_FILE:test_emt_delay>)

add_executable(test_emt_state_space runStateSpaceTests.cpp)
target_link_libraries(
  test_emt_state_space GridKit::emt_state_space GridKit::testing)

add_test(NAME EMTStateSpaceTest COMMAND $<TARGET_FILE:test_emt_state_space>)

install(TARGETS test_emt_delay test_emt_state_space RUNTIME DESTINATION bin)

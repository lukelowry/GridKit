add_executable(test_emt_delay runDelayTests.cpp)
target_link_libraries(
  test_emt_delay GridKit::emt_delay GridKit::testing)

add_test(NAME EMTDelayTest COMMAND $<TARGET_FILE:test_emt_delay>)

install(TARGETS test_emt_delay RUNTIME DESTINATION bin)

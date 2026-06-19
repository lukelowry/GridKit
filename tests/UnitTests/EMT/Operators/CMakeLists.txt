add_executable(test_emt_delay runDelayTests.cpp)
target_link_libraries(
  test_emt_delay GridKit::emt_delay GridKit::testing)

add_test(NAME EMTDelayTest COMMAND $<TARGET_FILE:test_emt_delay>)

add_executable(test_emt_state_space runStateSpaceTests.cpp)
target_link_libraries(
  test_emt_state_space GridKit::emt_state_space GridKit::testing)

add_test(NAME EMTStateSpaceTest COMMAND $<TARGET_FILE:test_emt_state_space>)

add_executable(test_emt_propagation runPropagationTests.cpp)
target_link_libraries(
  test_emt_propagation GridKit::emt_propagation GridKit::testing)

add_test(NAME EMTPropagationTest COMMAND $<TARGET_FILE:test_emt_propagation>)

add_executable(test_emt_propagation_parser runPropagationParserTests.cpp)
target_link_libraries(
  test_emt_propagation_parser GridKit::emt_propagation GridKit::testing)
target_include_directories(
  test_emt_propagation_parser
  PRIVATE ${GRIDKIT_THIRD_PARTY_DIR}/nlohmann-json/include)

add_test(NAME EMTPropagationParserTest COMMAND $<TARGET_FILE:test_emt_propagation_parser>)

install(
  TARGETS test_emt_delay
          test_emt_state_space
          test_emt_propagation
          test_emt_propagation_parser
  RUNTIME DESTINATION bin)

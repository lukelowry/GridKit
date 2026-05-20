add_executable(test_emt_architecture runEMTArchitectureTests.cpp)
target_link_libraries(test_emt_architecture
  GridKit::emt_components
  GridKit::testing)

add_executable(test_emt_json_parser runEMTJsonParserTests.cpp)
target_link_libraries(test_emt_json_parser
  GridKit::emt_components
  GridKit::testing)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/TwoBus.case.json
  ${CMAKE_CURRENT_BINARY_DIR}/TwoBus.case.json
  COPYONLY)

add_test(NAME EMTArchitectureTest COMMAND test_emt_architecture)
add_test(NAME EMTJsonParserTest COMMAND test_emt_json_parser
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

install(TARGETS test_emt_architecture
                test_emt_json_parser
        RUNTIME DESTINATION bin)

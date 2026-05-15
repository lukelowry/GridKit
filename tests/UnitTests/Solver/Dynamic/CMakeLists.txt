# 

add_executable(test_ida runIdaTests.cpp)
target_link_libraries(test_ida GridKit::solvers_dyn GridKit::solvers_dyn_io GridKit::testing)
target_include_directories(test_ida PRIVATE
  ${GRIDKIT_THIRD_PARTY_DIR}/nlohmann-json/include)
                                

add_test(NAME IDATest COMMAND $<TARGET_FILE:test_ida>)

install(TARGETS test_ida 
        RUNTIME DESTINATION bin)

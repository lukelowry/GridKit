# [[
#  Author(s):
#    - Luke Lowery <lukel@tamu.edu>
#]]

add_executable(test_case_data runCaseDataTests.cpp)
target_link_libraries(test_case_data GridKit::model_case
                                     GridKit::testing)

add_test(NAME CaseDataTest COMMAND test_case_data)

install(TARGETS test_case_data RUNTIME DESTINATION bin)

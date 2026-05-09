# [[
#  Author(s):
#    - Luke Lowery <lukel@tamu.edu>
#]]

add_executable(test_emt_rationalapprox runRationalApproxTests.cpp)
target_link_libraries(test_emt_rationalapprox GridKit::definitions
                                               GridKit::emt_rationalapprox
                                               GridKit::testing)

add_executable(test_emt_bus runBusTests.cpp)
target_link_libraries(test_emt_bus GridKit::definitions
                                   GridKit::emt_bus
                                   GridKit::testing)

add_executable(test_emt_line runLineTests.cpp)
target_link_libraries(test_emt_line GridKit::definitions
                                    GridKit::emt_line
                                    GridKit::testing)

add_executable(test_emt_load runShuntLoadTests.cpp)
target_link_libraries(test_emt_load GridKit::definitions
                                    GridKit::emt_load
                                    GridKit::emt_bus
                                    GridKit::testing)

add_executable(test_emt_voltage_source runVoltageSourceTests.cpp)
target_link_libraries(test_emt_voltage_source GridKit::definitions
                                               GridKit::emt_voltage_source
                                               GridKit::emt_bus
                                               GridKit::testing)

add_executable(test_emt_system runSystemModelTests.cpp)
target_link_libraries(test_emt_system GridKit::definitions
                                      GridKit::emt_bus
                                      GridKit::emt_components
                                      GridKit::testing)

add_executable(test_emt_system_model_data runSystemModelDataTests.cpp)
target_link_libraries(test_emt_system_model_data GridKit::definitions
                                                 GridKit::emt_components
                                                 GridKit::testing)

add_test(NAME EMTRationalApproxTest COMMAND test_emt_rationalapprox)
add_test(NAME EMTBusTest COMMAND test_emt_bus)
add_test(NAME EMTLineTest COMMAND test_emt_line)
add_test(NAME EMTShuntLoadTest COMMAND test_emt_load)
add_test(NAME EMTVoltageSourceTest COMMAND test_emt_voltage_source)
add_test(NAME EMTSystemModelTest COMMAND test_emt_system)
add_test(NAME EMTSystemModelDataTest COMMAND test_emt_system_model_data)

install(TARGETS test_emt_rationalapprox
                test_emt_bus
                test_emt_line
                test_emt_load
                test_emt_voltage_source
                test_emt_system
                test_emt_system_model_data
        RUNTIME DESTINATION bin)

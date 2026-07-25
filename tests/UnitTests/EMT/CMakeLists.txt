configure_file(${PROJECT_SOURCE_DIR}/examples/EMT/ThreeBus/IEEE13ReducedABC.case.json
               ${CMAKE_CURRENT_BINARY_DIR}/IEEE13ReducedABC.case.json COPYONLY)

set(_emt_test_targets
    test_emt_system
    test_emt_bus
    test_emt_linelumped
    test_emt_loadz
    test_emt_voltage_source
    test_emt_vectorfit)

add_executable(test_emt_system runSystemTests.cpp)
add_executable(test_emt_bus runBusTests.cpp)
add_executable(test_emt_linelumped runLineLumpedTests.cpp)
add_executable(test_emt_loadz runLoadZTests.cpp)
add_executable(test_emt_voltage_source runVoltageSourceTests.cpp)
add_executable(test_emt_vectorfit runVectorFitTests.cpp)

target_include_directories(
  test_emt_system
  PRIVATE ${GRIDKIT_THIRD_PARTY_DIR}/nlohmann-json/include ${PROJECT_SOURCE_DIR}/application/EMT)

foreach(_target IN LISTS _emt_test_targets)
  target_include_directories(${_target} PRIVATE ${GRIDKIT_THIRD_PARTY_DIR}/magic-enum/include)
  target_compile_definitions(
    ${_target} PRIVATE EMT_TEST_FIXTURE="${CMAKE_CURRENT_BINARY_DIR}/IEEE13ReducedABC.case.json")
  target_link_libraries(
    ${_target}
    GridKit::definitions
    GridKit::emt_systemmodel
    GridKit::emt_systemmodel_dependency_tracking
    GridKit::testing)
endforeach()

add_test(NAME EMTSystemTest COMMAND test_emt_system)
add_test(NAME EMTBusTest COMMAND test_emt_bus)
add_test(NAME EMTLineLumpedTest COMMAND test_emt_linelumped)
add_test(NAME EMTLoadZTest COMMAND test_emt_loadz)
add_test(NAME EMTVoltageSourceTest COMMAND test_emt_voltage_source)
add_test(NAME EMTVectorFitTest COMMAND test_emt_vectorfit)

install(TARGETS ${_emt_test_targets} RUNTIME DESTINATION bin)

add_executable(test_emt_bus runBusTests.cpp)
target_link_libraries(
  test_emt_bus
  GridKit::emt_bus
  GridKit::emt_bus_dependency_tracking
  GridKit::phasor_dynamics_systemmodel
  GridKit::testing)

add_test(NAME EMTBusTest COMMAND test_emt_bus)

install(TARGETS test_emt_bus RUNTIME DESTINATION bin)

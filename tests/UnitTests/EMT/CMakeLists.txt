add_executable(test_emt_bus runBusTests.cpp)
target_link_libraries(
  test_emt_bus
  GridKit::emt_component
  GridKit::emt_bus
  GridKit::emt_bus_dependency_tracking
  GridKit::phasor_dynamics_systemmodel
  GridKit::testing)

add_test(NAME EMTBusTest COMMAND test_emt_bus)

add_executable(test_emt_vectorfit runVectorFitTests.cpp)
target_link_libraries(
  test_emt_vectorfit
  GridKit::emt_vectorfit
  GridKit::phasor_dynamics_signal
  GridKit::testing)

if(GRIDKIT_ENABLE_ENZYME)
  target_link_libraries(test_emt_vectorfit ClangEnzymeFlags)
  target_compile_options(test_emt_vectorfit PRIVATE -mllvm -enzyme-auto-sparsity=1 -fno-math-errno)
endif()

add_test(NAME EMTVectorFitTest COMMAND test_emt_vectorfit)

install(TARGETS test_emt_bus test_emt_vectorfit RUNTIME DESTINATION bin)

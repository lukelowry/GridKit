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

add_executable(test_emt_linelumped runLineLumpedTests.cpp)
target_link_libraries(
  test_emt_linelumped
  GridKit::emt_line_lumped
  GridKit::phasor_dynamics_systemmodel
  GridKit::testing)

if(GRIDKIT_ENABLE_ENZYME)
  target_link_libraries(test_emt_linelumped ClangEnzymeFlags)
  target_compile_options(test_emt_linelumped PRIVATE -mllvm -enzyme-auto-sparsity=1 -fno-math-errno)
endif()

add_test(NAME EMTLineLumpedTest COMMAND test_emt_linelumped)

add_executable(test_emt_voltage_source runVoltageSourceTests.cpp)
target_link_libraries(
  test_emt_voltage_source
  GridKit::emt_voltage_source
  GridKit::phasor_dynamics_systemmodel
  GridKit::testing)

if(GRIDKIT_ENABLE_ENZYME)
  target_link_libraries(test_emt_voltage_source ClangEnzymeFlags)
  target_compile_options(test_emt_voltage_source PRIVATE -mllvm -enzyme-auto-sparsity=1
                                                         -fno-math-errno)
endif()

add_test(NAME EMTVoltageSourceTest COMMAND test_emt_voltage_source)

add_executable(test_emt_loadrl runLoadRLTests.cpp)
target_link_libraries(
  test_emt_loadrl
  GridKit::emt_load_rl
  GridKit::emt_voltage_source
  GridKit::phasor_dynamics_systemmodel
  GridKit::testing)

if(GRIDKIT_ENABLE_ENZYME)
  target_link_libraries(test_emt_loadrl ClangEnzymeFlags)
  target_compile_options(test_emt_loadrl PRIVATE -mllvm -enzyme-auto-sparsity=1 -fno-math-errno)
endif()

add_test(NAME EMTLoadRLTest COMMAND test_emt_loadrl)

add_executable(test_emt_systemmodel runSystemModelTests.cpp)
target_link_libraries(
  test_emt_systemmodel GridKit::emt_systemmodel GridKit::testing)

if(GRIDKIT_ENABLE_ENZYME)
  target_link_libraries(test_emt_systemmodel ClangEnzymeFlags)
  target_compile_options(test_emt_systemmodel PRIVATE -mllvm -enzyme-auto-sparsity=1
                                                      -fno-math-errno)
endif()

add_test(NAME EMTSystemModelTest COMMAND test_emt_systemmodel)

install(
  TARGETS test_emt_bus
          test_emt_vectorfit
          test_emt_linelumped
          test_emt_voltage_source
          test_emt_loadrl
          test_emt_systemmodel
  RUNTIME DESTINATION bin)

add_executable(emt_dynamic_simulation DynamicSimulation.cpp)
set_target_properties(
  emt_dynamic_simulation
  PROPERTIES OUTPUT_NAME DynamicSimulation)

target_link_libraries(
  emt_dynamic_simulation
  PUBLIC GridKit::emt_systemmodel GridKit::solvers_dyn GridKit::Utilities)

target_include_directories(
  emt_dynamic_simulation
  PRIVATE ${GRIDKIT_THIRD_PARTY_DIR}/nlohmann-json/include)

if(GRIDKIT_ENABLE_ENZYME)
  target_link_libraries(emt_dynamic_simulation PUBLIC ClangEnzymeFlags)
  target_compile_options(emt_dynamic_simulation PRIVATE -mllvm -enzyme-auto-sparsity=1
                                                        -fno-math-errno)
endif()

install(
  TARGETS emt_dynamic_simulation
  EXPORT gridkit-targets
  RUNTIME DESTINATION bin/EMT)

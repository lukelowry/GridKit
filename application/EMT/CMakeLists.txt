add_executable(EMTDynamicSimulation EMTDynamicSimulation.cpp)
target_link_libraries(
  EMTDynamicSimulation
  PUBLIC GridKit::emt_systemmodel GridKit::solvers_dyn GridKit::Utilities)
target_include_directories(
  EMTDynamicSimulation
  PRIVATE ${GRIDKIT_THIRD_PARTY_DIR}/nlohmann-json/include)

install(
  TARGETS EMTDynamicSimulation
  EXPORT gridkit-targets
  RUNTIME)

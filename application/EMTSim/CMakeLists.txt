add_executable(EMTSim EMTSim.cpp)
target_link_libraries(EMTSim
  PUBLIC
    GridKit::emt_io
    GridKit::solvers_dyn
    GridKit::testing)

install(TARGETS EMTSim EXPORT gridkit-targets RUNTIME)

add_executable(EMTSim EMTSim.cpp)
target_link_libraries(EMTSim
    PUBLIC
    GridKit::emt_components
    GridKit::solvers_dyn
    GridKit::Utilities)
target_include_directories(EMTSim PRIVATE
    ${GRIDKIT_THIRD_PARTY_DIR}/nlohmann-json/include)

install(TARGETS EMTSim EXPORT gridkit-targets RUNTIME)

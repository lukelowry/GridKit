set(_install_headers
  LoadRL.hpp
  LoadRLData.hpp
  LoadRLImpl.hpp)

add_library(emt_load_rl INTERFACE)
target_include_directories(emt_load_rl
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>)
target_compile_features(emt_load_rl INTERFACE cxx_std_20)
target_link_libraries(emt_load_rl
  INTERFACE GridKit::emt_core
  INTERFACE GridKit::emt_bus)
add_library(GridKit::emt_load_rl ALIAS emt_load_rl)

add_library(emt_load_rl_dependency_tracking INTERFACE)
target_link_libraries(emt_load_rl_dependency_tracking
  INTERFACE GridKit::emt_load_rl)
add_library(GridKit::emt_load_rl_dependency_tracking ALIAS emt_load_rl_dependency_tracking)

target_link_libraries(emt_components
  INTERFACE GridKit::emt_load_rl)
target_link_libraries(emt_components_dependency_tracking
  INTERFACE GridKit::emt_load_rl_dependency_tracking)

install(TARGETS
  emt_load_rl
  emt_load_rl_dependency_tracking
  EXPORT gridkit-targets)
install(FILES ${_install_headers}
  DESTINATION include/GridKit/Model/EMT/Components/LoadRL)

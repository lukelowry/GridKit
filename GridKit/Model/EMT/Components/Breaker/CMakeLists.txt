set(_install_headers
  Breaker.hpp
  BreakerData.hpp
  BreakerImpl.hpp)

add_library(emt_breaker INTERFACE)
target_include_directories(emt_breaker
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>)
target_compile_features(emt_breaker INTERFACE cxx_std_20)
target_link_libraries(emt_breaker
  INTERFACE GridKit::emt_core
  INTERFACE GridKit::emt_bus)
add_library(GridKit::emt_breaker ALIAS emt_breaker)

add_library(emt_breaker_dependency_tracking INTERFACE)
target_link_libraries(emt_breaker_dependency_tracking
  INTERFACE GridKit::emt_breaker)
add_library(GridKit::emt_breaker_dependency_tracking ALIAS emt_breaker_dependency_tracking)

target_link_libraries(emt_components
  INTERFACE GridKit::emt_breaker)
target_link_libraries(emt_components_dependency_tracking
  INTERFACE GridKit::emt_breaker_dependency_tracking)

install(TARGETS
  emt_breaker
  emt_breaker_dependency_tracking
  EXPORT gridkit-targets)
install(FILES ${_install_headers}
  DESTINATION include/GridKit/Model/EMT/Components/Breaker)

set(_install_headers
  Bus.hpp
  BusData.hpp
  BusDataJSONParser.hpp
  BusImpl.hpp)

add_library(emt_bus INTERFACE)
target_include_directories(emt_bus
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>)
target_compile_features(emt_bus INTERFACE cxx_std_20)
target_link_libraries(emt_bus
  INTERFACE GridKit::emt_core)
add_library(GridKit::emt_bus ALIAS emt_bus)

add_library(emt_bus_dependency_tracking INTERFACE)
target_link_libraries(emt_bus_dependency_tracking
  INTERFACE GridKit::emt_bus)
add_library(GridKit::emt_bus_dependency_tracking ALIAS emt_bus_dependency_tracking)

target_link_libraries(emt_components
  INTERFACE GridKit::emt_bus)
target_link_libraries(emt_components_dependency_tracking
  INTERFACE GridKit::emt_bus_dependency_tracking)

install(TARGETS
  emt_bus
  emt_bus_dependency_tracking
  EXPORT gridkit-targets)
install(FILES ${_install_headers}
  DESTINATION include/GridKit/Model/EMT/Bus)

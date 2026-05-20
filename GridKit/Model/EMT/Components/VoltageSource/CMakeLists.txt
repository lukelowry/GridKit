set(_install_headers
  VoltageSource.hpp
  VoltageSourceData.hpp
  VoltageSourceImpl.hpp)

add_library(emt_voltage_source INTERFACE)
target_include_directories(emt_voltage_source
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>)
target_compile_features(emt_voltage_source INTERFACE cxx_std_20)
target_link_libraries(emt_voltage_source
  INTERFACE GridKit::emt_core
  INTERFACE GridKit::emt_bus)
add_library(GridKit::emt_voltage_source ALIAS emt_voltage_source)

add_library(emt_voltage_source_dependency_tracking INTERFACE)
target_link_libraries(emt_voltage_source_dependency_tracking
  INTERFACE GridKit::emt_voltage_source)
add_library(GridKit::emt_voltage_source_dependency_tracking ALIAS emt_voltage_source_dependency_tracking)

target_link_libraries(emt_components
  INTERFACE GridKit::emt_voltage_source)
target_link_libraries(emt_components_dependency_tracking
  INTERFACE GridKit::emt_voltage_source_dependency_tracking)

install(TARGETS
  emt_voltage_source
  emt_voltage_source_dependency_tracking
  EXPORT gridkit-targets)
install(FILES ${_install_headers}
  DESTINATION include/GridKit/Model/EMT/Components/VoltageSource)

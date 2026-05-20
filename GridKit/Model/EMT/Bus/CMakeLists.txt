set(_install_headers
  Bus.hpp
  BusData.hpp
  BusDataJSONParser.hpp)

gridkit_add_library(emt_bus
  SOURCES
    Bus.cpp
  HEADERS
    ${_install_headers}
  INCLUDE_DIRECTORIES
    PRIVATE ${GRIDKIT_THIRD_PARTY_DIR}/magic-enum/include
  LINK_LIBRARIES
    PUBLIC GridKit::emt_core)

gridkit_add_library(emt_bus_dependency_tracking
  SOURCES
    BusDependencyTracking.cpp
  INCLUDE_DIRECTORIES
    PRIVATE ${GRIDKIT_THIRD_PARTY_DIR}/magic-enum/include
  LINK_LIBRARIES
    PUBLIC GridKit::emt_core)

target_link_libraries(emt_components
  INTERFACE GridKit::emt_bus)
target_link_libraries(emt_components_dependency_tracking
  INTERFACE GridKit::emt_bus_dependency_tracking)

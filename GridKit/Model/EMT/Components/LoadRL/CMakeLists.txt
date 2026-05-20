set(_install_headers
  LoadRL.hpp
  LoadRLData.hpp)

gridkit_add_library(emt_load_rl
  SOURCES
    LoadRL.cpp
  HEADERS
    ${_install_headers}
  INCLUDE_DIRECTORIES
    PRIVATE ${GRIDKIT_THIRD_PARTY_DIR}/magic-enum/include
  LINK_LIBRARIES
    PUBLIC GridKit::emt_core
    PUBLIC GridKit::emt_bus)

gridkit_add_library(emt_load_rl_dependency_tracking
  SOURCES
    LoadRLDependencyTracking.cpp
  INCLUDE_DIRECTORIES
    PRIVATE ${GRIDKIT_THIRD_PARTY_DIR}/magic-enum/include
  LINK_LIBRARIES
    PUBLIC GridKit::emt_core
    PUBLIC GridKit::emt_bus_dependency_tracking)

target_link_libraries(emt_components
  INTERFACE GridKit::emt_load_rl)
target_link_libraries(emt_components_dependency_tracking
  INTERFACE GridKit::emt_load_rl_dependency_tracking)

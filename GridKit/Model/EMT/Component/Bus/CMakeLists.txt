set(_install_headers Bus.hpp)

if(GRIDKIT_ENABLE_ENZYME)
  gridkit_add_library(
    emt_bus
    SOURCES BusEnzyme.cpp
    HEADERS ${_install_headers}
    INCLUDE_DIRECTORIES PRIVATE ${GRIDKIT_THIRD_PARTY_DIR}/magic-enum/include
    LINK_LIBRARIES PUBLIC GridKit::phasor_dynamics_bus)
else()
  gridkit_add_library(
    emt_bus
    SOURCES Bus.cpp
    HEADERS ${_install_headers}
    INCLUDE_DIRECTORIES PRIVATE ${GRIDKIT_THIRD_PARTY_DIR}/magic-enum/include
    LINK_LIBRARIES PUBLIC GridKit::phasor_dynamics_bus)
endif()

gridkit_add_library(
  emt_bus_dependency_tracking
  SOURCES BusDependencyTracking.cpp
  INCLUDE_DIRECTORIES PRIVATE ${GRIDKIT_THIRD_PARTY_DIR}/magic-enum/include
  LINK_LIBRARIES PUBLIC GridKit::phasor_dynamics_bus_dependency_tracking)

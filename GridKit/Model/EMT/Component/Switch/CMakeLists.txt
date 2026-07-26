set(_install_headers Switch.hpp SwitchData.hpp)

gridkit_add_library(
  emt_switch
  SOURCES Switch.cpp
  HEADERS ${_install_headers}
  INCLUDE_DIRECTORIES PRIVATE ${GRIDKIT_THIRD_PARTY_DIR}/magic-enum/include
  LINK_LIBRARIES
    PUBLIC
    GridKit::phasor_dynamics_core
    PUBLIC
    GridKit::phasor_dynamics_signal
    PUBLIC
    GridKit::emt_bus)

gridkit_add_library(
  emt_switch_dependency_tracking
  SOURCES SwitchDependencyTracking.cpp
  INCLUDE_DIRECTORIES PRIVATE ${GRIDKIT_THIRD_PARTY_DIR}/magic-enum/include
  LINK_LIBRARIES
    PUBLIC
    GridKit::phasor_dynamics_core
    PUBLIC
    GridKit::phasor_dynamics_signal_dependency_tracking
    PUBLIC
    GridKit::emt_bus_dependency_tracking)

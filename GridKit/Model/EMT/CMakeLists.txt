gridkit_add_library(
  emt_component
  INTERFACE_TARGET
  HEADERS Component.hpp
  LINK_LIBRARIES INTERFACE GridKit::phasor_dynamics_core)

add_subdirectory(Component)

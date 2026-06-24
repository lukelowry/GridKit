add_library(emt_components INTERFACE)
add_library(GridKit::emt_components ALIAS emt_components)

gridkit_add_library(
  emt_component
  INTERFACE_TARGET
  HEADERS Component.hpp ComponentLibrary.hpp
  LINK_LIBRARIES INTERFACE GridKit::phasor_dynamics_core)

target_link_libraries(
  emt_components
  INTERFACE GridKit::emt_component)

add_subdirectory(Component)
add_subdirectory(Operators)

install(TARGETS emt_components EXPORT gridkit-targets)

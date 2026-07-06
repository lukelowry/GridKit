# [[
#  Author(s):
#    - Luke Lowery <lukel@tamu.edu>
#]]

gridkit_add_library(
  phasor_dynamics_stabilizer
  INTERFACE_TARGET
  HEADERS StabilizerFactory.hpp
  LINK_LIBRARIES INTERFACE GridKit::phasor_dynamics_core)

target_link_libraries(
  phasor_dynamics_components
  INTERFACE GridKit::phasor_dynamics_stabilizer)

add_subdirectory(IEEEST)

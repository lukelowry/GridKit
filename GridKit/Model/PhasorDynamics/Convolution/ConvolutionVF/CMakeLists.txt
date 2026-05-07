# [[
#  Author(s):
#    - Luke Lowery <lukel@tamu.edu>
# ]]

set(_install_headers
  ConvolutionVF.hpp
  ConvolutionVFData.hpp)

gridkit_add_library(phasor_dynamics_convolutionvf
  SOURCES
    ConvolutionVF.cpp
  HEADERS
    ${_install_headers}
  LINK_LIBRARIES
    PUBLIC GridKit::phasor_dynamics_core
    PUBLIC GridKit::phasor_dynamics_signal)

gridkit_add_library(phasor_dynamics_convolutionvf_dependency_tracking
  SOURCES
    ConvolutionVFDependencyTracking.cpp
  LINK_LIBRARIES
    PUBLIC GridKit::phasor_dynamics_core
    PUBLIC GridKit::phasor_dynamics_signal_dependency_tracking)

# Link to interface target for all components
target_link_libraries(phasor_dynamics_components
  INTERFACE GridKit::phasor_dynamics_convolutionvf)
target_link_libraries(phasor_dynamics_components_dependency_tracking
  INTERFACE GridKit::phasor_dynamics_convolutionvf_dependency_tracking)

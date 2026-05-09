# [[
#  Author(s):
#    - Luke Lowery <lukel@tamu.edu>
# ]]

set(_install_headers
  CsvSignalSource.hpp
  CsvSignalSourceData.hpp)

gridkit_add_library(phasor_dynamics_source_csvsignalsource
  SOURCES
    CsvSignalSource.cpp
  HEADERS
    ${_install_headers}
  LINK_LIBRARIES
    PUBLIC GridKit::phasor_dynamics_core
    PUBLIC GridKit::phasor_dynamics_signal)

gridkit_add_library(phasor_dynamics_source_csvsignalsource_dependency_tracking
  SOURCES
    CsvSignalSourceDependencyTracking.cpp
  LINK_LIBRARIES
    PUBLIC GridKit::phasor_dynamics_core
    PUBLIC GridKit::phasor_dynamics_signal_dependency_tracking)

target_link_libraries(phasor_dynamics_components
  INTERFACE GridKit::phasor_dynamics_source_csvsignalsource)
target_link_libraries(phasor_dynamics_components_dependency_tracking
  INTERFACE GridKit::phasor_dynamics_source_csvsignalsource_dependency_tracking)

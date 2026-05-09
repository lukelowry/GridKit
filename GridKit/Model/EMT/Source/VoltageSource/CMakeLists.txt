# [[
#  Author(s):
#    - Luke Lowery <lukel@tamu.edu>
#]]

set(_install_headers
  VoltageSource.hpp
  VoltageSourceData.hpp
  VoltageSourceImpl.hpp)

gridkit_add_library(emt_voltage_source
  SOURCES
    VoltageSource.cpp
  HEADERS
    ${_install_headers}
  LINK_LIBRARIES
    PUBLIC GridKit::emt_core)

target_link_libraries(emt_components
  INTERFACE GridKit::emt_voltage_source)

# [[
#  Author(s):
#    - Luke Lowery <lukel@tamu.edu>
#]]

set(_install_headers
  Bus.hpp
  BusData.hpp
  BusImpl.hpp)

gridkit_add_library(emt_bus
  SOURCES
    Bus.cpp
  HEADERS
    ${_install_headers}
  LINK_LIBRARIES
    PUBLIC GridKit::emt_core)

target_link_libraries(emt_components
  INTERFACE GridKit::emt_bus)

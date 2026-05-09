# [[
#  Author(s):
#    - Luke Lowery <lukel@tamu.edu>
#]]

set(_install_headers
  Line.hpp
  LineData.hpp
  LineImpl.hpp)

gridkit_add_library(emt_line
  SOURCES
    Line.cpp
  HEADERS
    ${_install_headers}
  LINK_LIBRARIES
    PUBLIC GridKit::emt_core
    PUBLIC GridKit::emt_bus
    PUBLIC GridKit::emt_rationalapprox)

target_link_libraries(emt_components
  INTERFACE GridKit::emt_line)

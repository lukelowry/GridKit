# [[
#  Author(s):
#    - Luke Lowery <lukel@tamu.edu>
#]]

set(_install_headers
  ShuntLoad.hpp
  ShuntLoadData.hpp
  ShuntLoadImpl.hpp)

gridkit_add_library(emt_load
  SOURCES
    ShuntLoad.cpp
  HEADERS
    ${_install_headers}
  LINK_LIBRARIES
    PUBLIC GridKit::emt_core)

target_link_libraries(emt_components
  INTERFACE GridKit::emt_load)

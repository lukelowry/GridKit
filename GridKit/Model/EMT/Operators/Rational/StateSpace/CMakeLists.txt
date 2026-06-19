gridkit_add_library(
  emt_state_space
  SOURCES StateSpace.cpp
  HEADERS StateSpace.hpp StateSpaceData.hpp StateSpaceDataJSONParser.hpp
  LINK_LIBRARIES PUBLIC GridKit::emt_core)

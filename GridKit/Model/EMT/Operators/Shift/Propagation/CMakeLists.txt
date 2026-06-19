gridkit_add_library(
  emt_propagation
  SOURCES Propagation.cpp
  HEADERS Propagation.hpp PropagationData.hpp PropagationDataJSONParser.hpp
  LINK_LIBRARIES PUBLIC GridKit::emt_delay GridKit::emt_state_space)

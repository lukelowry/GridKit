gridkit_add_library(
  emt_delay
  SOURCES Delay.cpp
  HEADERS Delay.hpp
  LINK_LIBRARIES PUBLIC GridKit::sparse_matrix)

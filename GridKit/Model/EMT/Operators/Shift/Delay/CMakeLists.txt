gridkit_add_library(
  emt_delay
  SOURCES Delay.cpp
  HEADERS Delay.hpp DelayData.hpp
  LINK_LIBRARIES PUBLIC GridKit::emt_core)

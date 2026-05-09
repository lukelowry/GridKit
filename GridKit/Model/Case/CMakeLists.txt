# [[
#  Author(s):
#    - Luke Lowery <lukel@tamu.edu>
#]]

gridkit_add_library(model_case
  SOURCES
    CaseData.cpp
  HEADERS
    CaseData.hpp
  LINK_LIBRARIES
    PUBLIC GridKit::definitions
  INCLUDE_DIRECTORIES
    PUBLIC $<BUILD_INTERFACE:${GRIDKIT_THIRD_PARTY_DIR}/nlohmann-json/include>)

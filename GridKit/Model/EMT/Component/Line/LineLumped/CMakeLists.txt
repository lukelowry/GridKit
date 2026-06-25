gridkit_add_library(
  emt_line_lumped
  INTERFACE_TARGET
  HEADERS LineLumped.hpp
          LineLumpedImpl.hpp
          LineLumpedEnzyme.hpp
          LineLumpedData.hpp
          LineLumpedDataJSONParser.hpp
  LINK_LIBRARIES
    INTERFACE
    GridKit::emt_component
    GridKit::emt_bus
    GridKit::emt_vectorfit
  INCLUDE_DIRECTORIES INTERFACE $<BUILD_INTERFACE:${GRIDKIT_THIRD_PARTY_DIR}/nlohmann-json/include>)

target_link_libraries(
  emt_components
  INTERFACE GridKit::emt_line_lumped)

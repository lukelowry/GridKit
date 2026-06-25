gridkit_add_library(
  emt_load_rl
  INTERFACE_TARGET
  HEADERS LoadRL.hpp
          LoadRLImpl.hpp
          LoadRLEnzyme.hpp
          LoadRLData.hpp
          LoadRLDataJSONParser.hpp
  LINK_LIBRARIES INTERFACE GridKit::emt_component GridKit::emt_bus
  INCLUDE_DIRECTORIES INTERFACE $<BUILD_INTERFACE:${GRIDKIT_THIRD_PARTY_DIR}/nlohmann-json/include>)

target_link_libraries(
  emt_components
  INTERFACE GridKit::emt_load_rl)

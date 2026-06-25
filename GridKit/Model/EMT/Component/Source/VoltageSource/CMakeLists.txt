gridkit_add_library(
  emt_voltage_source
  INTERFACE_TARGET
  HEADERS VoltageSource.hpp
          VoltageSourceImpl.hpp
          VoltageSourceEnzyme.hpp
          VoltageSourceData.hpp
          VoltageSourceDataJSONParser.hpp
  LINK_LIBRARIES INTERFACE GridKit::emt_component GridKit::emt_bus
  INCLUDE_DIRECTORIES INTERFACE $<BUILD_INTERFACE:${GRIDKIT_THIRD_PARTY_DIR}/nlohmann-json/include>)

target_link_libraries(
  emt_components
  INTERFACE GridKit::emt_voltage_source)

add_library(emt_components INTERFACE)
add_library(GridKit::emt_components ALIAS emt_components)

gridkit_add_library(
  emt_component
  INTERFACE_TARGET
  HEADERS Component.hpp ComponentLibrary.hpp ComponentDataJSONParser.hpp
  LINK_LIBRARIES INTERFACE GridKit::phasor_dynamics_core
  INCLUDE_DIRECTORIES
    INTERFACE
    $<BUILD_INTERFACE:${GRIDKIT_THIRD_PARTY_DIR}/nlohmann-json/include>
    INTERFACE
    $<BUILD_INTERFACE:${GRIDKIT_THIRD_PARTY_DIR}/magic-enum/include>)

target_link_libraries(
  emt_components
  INTERFACE GridKit::emt_component)

add_subdirectory(Component)
add_subdirectory(Operators)

gridkit_add_library(
  emt_systemmodel
  SOURCES SystemModelData.cpp
  HEADERS SystemModel.hpp SystemModelData.hpp SystemModelDataJSONParser.hpp
  LINK_LIBRARIES
    PUBLIC
    GridKit::emt_components
    PUBLIC
    GridKit::phasor_dynamics_systemmodel
  INCLUDE_DIRECTORIES PUBLIC $<BUILD_INTERFACE:${GRIDKIT_THIRD_PARTY_DIR}/nlohmann-json/include>)

install(TARGETS emt_components EXPORT gridkit-targets)

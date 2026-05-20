# [[
#  EMT simple architecture skeleton
#]]

add_library(emt_components INTERFACE)
add_library(emt_components_dependency_tracking INTERFACE)

gridkit_add_library(emt_core
  SOURCES
    SystemModelData.cpp
  HEADERS
    Component.hpp
    ComponentData.hpp
    ComponentDataJSONParser.hpp
    ComponentLibrary.hpp
    GridElement.hpp
    SystemModel.hpp
    SystemModelData.hpp
    SystemModelDataJSONParser.hpp
  LINK_LIBRARIES
    PUBLIC GridKit::definitions
    PUBLIC GridKit::utilities_logger
    PUBLIC GridKit::sparse_matrix
  INCLUDE_DIRECTORIES
    PRIVATE ${GRIDKIT_THIRD_PARTY_DIR}/nlohmann-json/include
    PRIVATE ${GRIDKIT_THIRD_PARTY_DIR}/magic-enum/include)

target_link_libraries(emt_components
  INTERFACE GridKit::emt_core)
target_link_libraries(emt_components_dependency_tracking
  INTERFACE GridKit::emt_core)

add_subdirectory(Bus)
add_subdirectory(Components/BranchLumpedConstant)
add_subdirectory(Components/LoadRL)
add_subdirectory(Components/VoltageSource)
add_subdirectory(Components/Breaker)
add_subdirectory(Math/RationalApprox)

add_library(GridKit::emt_components
  ALIAS emt_components)
add_library(GridKit::emt_components_dependency_tracking
  ALIAS emt_components_dependency_tracking)

install(TARGETS
  emt_components
  emt_components_dependency_tracking
  EXPORT gridkit-targets)

install(FILES
  Math/PhaseMath.hpp
  DESTINATION include/GridKit/Model/EMT/Math)

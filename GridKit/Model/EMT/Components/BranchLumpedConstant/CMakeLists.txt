set(_install_headers
  BranchLumpedConstant.hpp
  BranchLumpedConstantData.hpp)

gridkit_add_library(emt_branch_lumped_constant
  SOURCES
    BranchLumpedConstant.cpp
  HEADERS
    ${_install_headers}
  INCLUDE_DIRECTORIES
    PRIVATE ${GRIDKIT_THIRD_PARTY_DIR}/magic-enum/include
  LINK_LIBRARIES
    PUBLIC GridKit::emt_core
    PUBLIC GridKit::emt_bus)

gridkit_add_library(emt_branch_lumped_constant_dependency_tracking
  SOURCES
    BranchLumpedConstantDependencyTracking.cpp
  INCLUDE_DIRECTORIES
    PRIVATE ${GRIDKIT_THIRD_PARTY_DIR}/magic-enum/include
  LINK_LIBRARIES
    PUBLIC GridKit::emt_core
    PUBLIC GridKit::emt_bus_dependency_tracking)

target_link_libraries(emt_components
  INTERFACE GridKit::emt_branch_lumped_constant)
target_link_libraries(emt_components_dependency_tracking
  INTERFACE GridKit::emt_branch_lumped_constant_dependency_tracking)

set(_install_headers
  BranchLumpedConstant.hpp
  BranchLumpedConstantData.hpp
  BranchLumpedConstantImpl.hpp)

add_library(emt_branch_lumped_constant INTERFACE)
target_include_directories(emt_branch_lumped_constant
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>)
target_compile_features(emt_branch_lumped_constant INTERFACE cxx_std_20)
target_link_libraries(emt_branch_lumped_constant
  INTERFACE GridKit::emt_core
  INTERFACE GridKit::emt_bus)
add_library(GridKit::emt_branch_lumped_constant ALIAS emt_branch_lumped_constant)

add_library(emt_branch_lumped_constant_dependency_tracking INTERFACE)
target_link_libraries(emt_branch_lumped_constant_dependency_tracking
  INTERFACE GridKit::emt_branch_lumped_constant)
add_library(GridKit::emt_branch_lumped_constant_dependency_tracking ALIAS emt_branch_lumped_constant_dependency_tracking)

target_link_libraries(emt_components
  INTERFACE GridKit::emt_branch_lumped_constant)
target_link_libraries(emt_components_dependency_tracking
  INTERFACE GridKit::emt_branch_lumped_constant_dependency_tracking)

install(TARGETS
  emt_branch_lumped_constant
  emt_branch_lumped_constant_dependency_tracking
  EXPORT gridkit-targets)
install(FILES ${_install_headers}
  DESTINATION include/GridKit/Model/EMT/Components/BranchLumpedConstant)

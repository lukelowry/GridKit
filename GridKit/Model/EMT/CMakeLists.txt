if(NOT GRIDKIT_ENABLE_ENZYME)
  message(FATAL_ERROR "GridKit EMT requires GRIDKIT_ENABLE_ENZYME=ON")
endif()

add_library(emt INTERFACE)
target_include_directories(emt
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>)
target_compile_features(emt INTERFACE cxx_std_20)
target_link_libraries(emt
  INTERFACE
    GridKit::definitions
    GridKit::sparse_matrix
    ClangEnzymeFlags)
target_compile_options(emt
  INTERFACE
    -mllvm -enzyme-auto-sparsity=1
    -fno-math-errno)

add_library(GridKit::emt ALIAS emt)

install(TARGETS emt EXPORT gridkit-targets)
install(
  FILES
    README.md
    PhaseMath.hpp
    SystemModel.hpp
  DESTINATION
    include/GridKit/Model/EMT)
install(
  FILES
    Bus/Bus.hpp
    Bus/BusData.hpp
  DESTINATION
    include/GridKit/Model/EMT/Bus)
install(
  FILES
    Branch/BranchLumpedConstant/BranchLumpedConstant.hpp
    Branch/BranchLumpedConstant/BranchLumpedConstantData.hpp
  DESTINATION
    include/GridKit/Model/EMT/Branch/BranchLumpedConstant)
install(
  FILES
    Component/LoadRL/LoadRL.hpp
    Component/LoadRL/LoadRLData.hpp
  DESTINATION
    include/GridKit/Model/EMT/Component/LoadRL)
install(
  FILES
    Component/VoltageSource/VoltageSource.hpp
    Component/VoltageSource/VoltageSourceData.hpp
  DESTINATION
    include/GridKit/Model/EMT/Component/VoltageSource)
install(
  FILES
    System/Jacobian.hpp
    System/Layout.hpp
    System/Monitor.hpp
    System/Network.hpp
    System/Views.hpp
  DESTINATION
    include/GridKit/Model/EMT/System)

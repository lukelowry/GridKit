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

add_library(emt_io INTERFACE)
target_link_libraries(emt_io
  INTERFACE
    GridKit::emt)
target_include_directories(emt_io
  INTERFACE
    $<BUILD_INTERFACE:${GRIDKIT_THIRD_PARTY_DIR}/nlohmann-json/include>
    $<INSTALL_INTERFACE:include>)

add_library(GridKit::emt_io ALIAS emt_io)

install(TARGETS emt EXPORT gridkit-targets)
install(TARGETS emt_io EXPORT gridkit-targets)
install(
  FILES
    Case.hpp
    ComponentLibrary.hpp
    README.md
    PhaseMath.hpp
    SystemModel.hpp
    SystemModelData.hpp
  DESTINATION
    include/GridKit/Model/EMT)
install(
  FILES
    Bus/Bus.hpp
  DESTINATION
    include/GridKit/Model/EMT/Bus)
install(
  FILES
    Component/Branch/BranchLumpedConstant/BranchLumpedConstant.hpp
  DESTINATION
    include/GridKit/Model/EMT/Component/Branch/BranchLumpedConstant)
install(
  FILES
    Component/LoadRL/LoadRL.hpp
  DESTINATION
    include/GridKit/Model/EMT/Component/LoadRL)
install(
  FILES
    Component/VoltageSource/VoltageSource.hpp
  DESTINATION
    include/GridKit/Model/EMT/Component/VoltageSource)
install(
  FILES
    Component/Breaker/Breaker.hpp
  DESTINATION
    include/GridKit/Model/EMT/Component/Breaker)
install(
  FILES
    System/ComponentDescriptor.hpp
    System/Components.hpp
    System/Events.hpp
    System/Jacobian.hpp
    System/Layout.hpp
    System/Monitor.hpp
    System/Views.hpp
  DESTINATION
    include/GridKit/Model/EMT/System)
install(
  FILES
    IO/JsonSupport.hpp
    IO/ParamReader.hpp
  DESTINATION
    include/GridKit/Model/EMT/IO)

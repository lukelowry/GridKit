add_library(emt INTERFACE)
target_include_directories(emt
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>)
target_compile_features(emt INTERFACE cxx_std_20)
target_link_libraries(emt
  INTERFACE
    GridKit::definitions
    GridKit::sparse_matrix)

add_library(GridKit::emt ALIAS emt)

install(TARGETS emt EXPORT gridkit-targets)
install(
  FILES
    README.md
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
    System/Assembly.hpp
    System/Component.hpp
    System/ComponentStore.hpp
    System/ComponentTraits.hpp
    System/EnzymeJacobian.hpp
    System/JacobianView.hpp
    System/Layout.hpp
    System/NetworkData.hpp
    System/PatternView.hpp
    System/Port.hpp
    System/ResidualView.hpp
    System/Terminal.hpp
    System/VariableView.hpp
  DESTINATION
    include/GridKit/Model/EMT/System)

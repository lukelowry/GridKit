# [[
#  Author(s):
#    - Luke Lowery <lukel@tamu.edu>
#]]

add_library(emt_core INTERFACE)
add_library(emt_components INTERFACE)

target_include_directories(emt_core
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(emt_core
  INTERFACE GridKit::definitions
  INTERFACE GridKit::utilities_logger
  INTERFACE GridKit::sparse_matrix)
target_compile_features(emt_core INTERFACE cxx_std_20)

add_library(GridKit::emt_core ALIAS emt_core)
add_library(GridKit::emt_components ALIAS emt_components)

gridkit_add_library(emt_system_model_data
  SOURCES
    SystemModelData.cpp
  HEADERS
    SystemModelData.hpp
    SystemModelDataJSONParser.hpp
  LINK_LIBRARIES
    PUBLIC GridKit::emt_core
    PUBLIC GridKit::model_case)

target_link_libraries(emt_components
  INTERFACE GridKit::emt_core
  INTERFACE GridKit::emt_system_model_data)

add_subdirectory(RationalApprox)
add_subdirectory(Bus)
add_subdirectory(Line)
add_subdirectory(Load)
add_subdirectory(Source/VoltageSource)

install(TARGETS
  emt_core
  emt_components
  EXPORT gridkit-targets)

install(FILES
  Component.hpp
  SystemModel.hpp
  SystemModelImpl.hpp
  DESTINATION include/GridKit/Model/EMT)

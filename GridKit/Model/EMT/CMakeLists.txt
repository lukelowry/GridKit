add_library(emt_components INTERFACE)
add_library(emt_components_dependency_tracking INTERFACE)

add_subdirectory(Bus)
add_subdirectory(Component/Line/LineLumped)
add_subdirectory(Component/Load/LoadZ)
add_subdirectory(Component/Source/VoltageSource)
add_subdirectory(Component/Switch)
add_subdirectory(Operators/Rational/VectorFit)

target_link_libraries(
  emt_components
  INTERFACE GridKit::emt_bus
            GridKit::emt_linelumped
            GridKit::emt_loadz
            GridKit::emt_voltage_source
            GridKit::emt_switch
            GridKit::emt_vectorfit
            GridKit::phasor_dynamics_signalsource)
target_link_libraries(
  emt_components_dependency_tracking
  INTERFACE GridKit::emt_bus_dependency_tracking
            GridKit::emt_linelumped_dependency_tracking
            GridKit::emt_loadz_dependency_tracking
            GridKit::emt_voltage_source_dependency_tracking
            GridKit::emt_switch_dependency_tracking
            GridKit::emt_vectorfit_dependency_tracking
            GridKit::phasor_dynamics_signalsource_dependency_tracking)

add_library(GridKit::emt_components ALIAS emt_components)
add_library(GridKit::emt_components_dependency_tracking ALIAS emt_components_dependency_tracking)

if(GRIDKIT_ENABLE_ENZYME)
  set(_system_model_source SystemModelEnzyme.cpp)
else()
  set(_system_model_source SystemModel.cpp)
endif()

gridkit_add_library(
  emt_systemmodel
  SOURCES ${_system_model_source} SystemModelData.cpp
  HEADERS SystemModel.hpp
          SystemModelData.hpp
          SystemModelDataJSONParser.hpp
          BusVoltageContribution.hpp
          InitialStateLayout.hpp
          ComponentLibrary.hpp
          ComponentData.hpp
          ABCUtils.hpp
  LINK_LIBRARIES
    PUBLIC
    GridKit::phasor_dynamics_core
    PUBLIC
    GridKit::emt_components
  INCLUDE_DIRECTORIES PRIVATE ${GRIDKIT_THIRD_PARTY_DIR}/nlohmann-json/include)

gridkit_add_library(
  emt_systemmodel_dependency_tracking
  SOURCES SystemModelDependencyTracking.cpp
  HEADERS SystemModel.hpp
  LINK_LIBRARIES
    PUBLIC
    GridKit::phasor_dynamics_core
    PUBLIC
    GridKit::emt_components_dependency_tracking)

install(TARGETS emt_components emt_components_dependency_tracking EXPORT gridkit-targets)

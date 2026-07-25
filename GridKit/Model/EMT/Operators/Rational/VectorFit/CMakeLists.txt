set(_vectorfit_headers VectorFit.hpp VectorFitData.hpp)

if(GRIDKIT_ENABLE_ENZYME)
  set(_vectorfit_source VectorFitEnzyme.cpp)
else()
  set(_vectorfit_source VectorFit.cpp)
endif()

gridkit_add_library(
  emt_vectorfit
  SOURCES ${_vectorfit_source}
  HEADERS ${_vectorfit_headers}
  LINK_LIBRARIES
    PUBLIC
    GridKit::phasor_dynamics_core
    PUBLIC
    GridKit::phasor_dynamics_signal)

gridkit_add_library(
  emt_vectorfit_dependency_tracking
  SOURCES VectorFitDependencyTracking.cpp
  LINK_LIBRARIES
    PUBLIC
    GridKit::phasor_dynamics_core
    PUBLIC
    GridKit::phasor_dynamics_signal_dependency_tracking)

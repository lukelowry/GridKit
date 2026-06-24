gridkit_add_library(
  emt_vectorfit
  INTERFACE_TARGET
  HEADERS VectorFit.hpp
          VectorFitImpl.hpp
          VectorFitEnzyme.hpp
          VectorFitData.hpp
          VectorFitDataJSONParser.hpp
  LINK_LIBRARIES INTERFACE GridKit::emt_component GridKit::phasor_dynamics_signal
  INCLUDE_DIRECTORIES INTERFACE $<BUILD_INTERFACE:${GRIDKIT_THIRD_PARTY_DIR}/nlohmann-json/include>)

target_link_libraries(
  emt_components
  INTERFACE GridKit::emt_vectorfit)

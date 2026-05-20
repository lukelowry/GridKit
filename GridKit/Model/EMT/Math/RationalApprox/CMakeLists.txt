add_library(emt_rational_approx INTERFACE)

target_include_directories(emt_rational_approx
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(emt_rational_approx
  INTERFACE GridKit::emt_core)
target_compile_features(emt_rational_approx INTERFACE cxx_std_20)

add_library(GridKit::emt_rational_approx ALIAS emt_rational_approx)

target_link_libraries(emt_components
  INTERFACE GridKit::emt_rational_approx)

install(TARGETS
  emt_rational_approx
  EXPORT gridkit-targets)

install(FILES
  RationalApprox.hpp
  RationalApproxData.hpp
  RationalApproxImpl.hpp
  DESTINATION include/GridKit/Model/EMT/Math/RationalApprox)

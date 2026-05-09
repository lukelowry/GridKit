# [[
#  Author(s):
#    - Luke Lowery <lukel@tamu.edu>
#]]

add_library(emt_rationalapprox INTERFACE)

target_include_directories(emt_rationalapprox
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(emt_rationalapprox
  INTERFACE GridKit::emt_core)
target_compile_features(emt_rationalapprox INTERFACE cxx_std_20)

add_library(GridKit::emt_rationalapprox ALIAS emt_rationalapprox)

target_link_libraries(emt_components
  INTERFACE GridKit::emt_rationalapprox)

install(TARGETS
  emt_rationalapprox
  EXPORT gridkit-targets)

install(FILES
  RationalApprox.hpp
  RationalApproxData.hpp
  RationalApproxImpl.hpp
  DESTINATION include/GridKit/Model/EMT/RationalApprox)

gridkit_example_current_install_path(_install_path)

if(TARGET GridKit::solvers_dyn)
  add_executable(EMTTinyTwoBus TwoBus.cpp)
  target_link_libraries(EMTTinyTwoBus
    GridKit::emt
    GridKit::solvers_dyn)

  install(TARGETS EMTTinyTwoBus RUNTIME DESTINATION ${_install_path})

  add_test(NAME EMTTinyTwoBus COMMAND EMTTinyTwoBus
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
else()
  message(STATUS "Skipping EMTTinyTwoBus: requires GridKit::solvers_dyn")
endif()

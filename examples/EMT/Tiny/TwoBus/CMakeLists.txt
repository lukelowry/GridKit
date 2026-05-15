gridkit_example_current_install_path(_install_path)

if(TARGET GridKit::solvers_dyn)
  add_executable(EMTTinyTwoBus TwoBus.cpp)
  target_link_libraries(EMTTinyTwoBus
    GridKit::emt_io
    GridKit::solvers_dyn)
  gridkit_example_add_file(TwoBus.case.json)
  gridkit_example_add_file(TwoBus.solver.json)

  install(TARGETS EMTTinyTwoBus RUNTIME DESTINATION ${_install_path})

  add_test(NAME EMTTinyTwoBus COMMAND EMTTinyTwoBus
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

  if(TARGET EMTSim)
    add_test(NAME EMTSimTinyTwoBus COMMAND EMTSim TwoBus.solver.json
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  endif()

  add_executable(EMTTinyTwoBusManual TwoBusManual.cpp)
  target_link_libraries(EMTTinyTwoBusManual
    GridKit::emt
    GridKit::solvers_dyn)

  install(TARGETS EMTTinyTwoBusManual RUNTIME DESTINATION ${_install_path})

  add_test(NAME EMTTinyTwoBusManual COMMAND EMTTinyTwoBusManual
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

else()
  message(STATUS "Skipping EMTTinyTwoBus: requires GridKit::solvers_dyn")
endif()

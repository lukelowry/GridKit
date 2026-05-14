gridkit_example_current_install_path(_install_path)

if(TARGET GridKit::solvers_dyn)
  add_executable(EMTTinyTwoBus TwoBus.cpp)
  target_link_libraries(EMTTinyTwoBus
    GridKit::emt
    GridKit::solvers_dyn)

  install(TARGETS EMTTinyTwoBus RUNTIME DESTINATION ${_install_path})

  add_test(NAME EMTTinyTwoBus COMMAND EMTTinyTwoBus
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_Interpreter_FOUND)
    add_custom_target(EMTTinyTwoBusPlot
      COMMAND EMTTinyTwoBus
      COMMAND ${Python3_EXECUTABLE}
              ${CMAKE_SOURCE_DIR}/examples/EMT/plot_monitor_csv.py
              --input EMTTinyTwoBus.csv
              --output EMTTinyTwoBus.png
      DEPENDS EMTTinyTwoBus
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      COMMENT "Generating EMTTinyTwoBus.csv and EMTTinyTwoBus.png")
  endif()
else()
  message(STATUS "Skipping EMTTinyTwoBus: requires GridKit::solvers_dyn")
endif()

if(TARGET SUNDIALS::idas)
  add_subdirectory(PhasorDynamics)
  add_subdirectory(EMT)
endif()

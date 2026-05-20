if(TARGET SUNDIALS::idas)
  add_subdirectory(PhasorDynamics)
endif()

if(TARGET SUNDIALS::idas AND GRIDKIT_ENABLE_ENZYME)
  add_subdirectory(EMTSim)
endif()

if(TARGET SUNDIALS::idas)
  add_subdirectory(PhasorDynamics)
endif()

if(TARGET GridKit::emt_io AND TARGET GridKit::solvers_dyn)
  add_subdirectory(EMTSim)
endif()

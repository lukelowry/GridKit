gridkit_example_current_install_path(_install_path)

add_executable(ConvolutionVecDemo ConvolutionVecDemo.cpp)
target_link_libraries(ConvolutionVecDemo
  GridKit::phasor_dynamics_components
  GridKit::phasor_dynamics_signal
  GridKit::solvers_dyn)
install(TARGETS ConvolutionVecDemo RUNTIME DESTINATION ${_install_path})

gridkit_example_add_file(convolutionvec_three_phase_input.csv)

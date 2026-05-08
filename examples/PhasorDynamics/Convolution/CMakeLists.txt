gridkit_example_current_install_path(_install_path)

add_executable(ConvolutionVFDemo ConvolutionVFDemo.cpp)
target_link_libraries(ConvolutionVFDemo
  GridKit::phasor_dynamics_components
  GridKit::phasor_dynamics_signal
  GridKit::solvers_dyn)
install(TARGETS ConvolutionVFDemo RUNTIME DESTINATION ${_install_path})

gridkit_example_add_file(convolutionvf_input.csv)

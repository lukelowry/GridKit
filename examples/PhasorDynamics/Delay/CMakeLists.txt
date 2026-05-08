gridkit_example_current_install_path(_install_path)

add_executable(SignalDelayDemo SignalDelayDemo.cpp)
target_link_libraries(SignalDelayDemo
  GridKit::phasor_dynamics_components
  GridKit::phasor_dynamics_signal
  GridKit::solvers_dyn)
install(TARGETS SignalDelayDemo RUNTIME DESTINATION ${_install_path})

gridkit_example_add_file(signaldelay_input.csv)
